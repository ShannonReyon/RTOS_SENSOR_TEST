#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_timer.h>
#include "hyt939.h"
#include "tsic306.h"
#include "adis16407.h"

// ---------------- CONFIG ----------------
constexpr uint32_t HYT_PERIOD_MS = 120;      // ~8.33 Hz, leaves time for sensor conversion
constexpr uint32_t TSIC_PERIOD_MS = 100;     // 10 Hz
constexpr uint32_t ADIS_PERIOD_MS = 100;     // 10 Hz for now
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

constexpr uint32_t HYT_STACK_SIZE = 4096;
constexpr uint32_t TSIC_STACK_SIZE = 4096;
constexpr uint32_t ADIS_STACK_SIZE = 4096;
constexpr uint32_t GNSS_STACK_SIZE = 4096;
constexpr uint32_t LOGGER_STACK_SIZE = 4096;
constexpr uint32_t HEARTBEAT_STACK_SIZE = 2048;

constexpr UBaseType_t SENSOR_PRIORITY = 2;
constexpr UBaseType_t ADIS_PRIORITY = 3;
constexpr UBaseType_t GNSS_PRIORITY = 2;
constexpr UBaseType_t LOGGER_PRIORITY = 1;
constexpr UBaseType_t HEARTBEAT_PRIORITY = 1;

constexpr BaseType_t CORE = 1;

constexpr uint8_t HYT_I2C_ADDRESS = 0x28;
constexpr size_t HYT_QUEUE_LENGTH = 10;
constexpr size_t TSIC_QUEUE_LENGTH = 10;
constexpr size_t ADIS_QUEUE_LENGTH = 10;

// Set this to your actual TSIC data pin
constexpr int TSIC_DATA_PIN = 4;

// Set this to your actual ADIS chip-select pin
constexpr int ADIS_CS_PIN = 5;

// GNSS UART settings: GNSS TX -> ESP32 GPIO16 (RX2), GNSS RX -> ESP32 GPIO17 (TX2 optional)
constexpr uint32_t GNSS_BAUD = 230400;
constexpr int GNSS_RX_PIN = 16;
constexpr int GNSS_TX_PIN = 17;

// UBX NAV-PVT message constants
constexpr uint8_t UBX_SYNC_1 = 0xB5;
constexpr uint8_t UBX_SYNC_2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
constexpr uint16_t NAV_PVT_PAYLOAD_LEN = 92;

// ---------------- DATA TYPES ----------------
struct HYTSample
{
    uint32_t timestamp_ms;
    uint8_t status;
    float humidity_percent;
    float temperature_c;
    bool valid;
};

struct TSICSample
{
    uint32_t timestamp_ms;
    uint16_t raw11;
    uint32_t tStrobe_us;
    float temperature_c;
    bool valid;
};

struct ADISSample
{
    uint32_t timestamp_ms;
    float temperature_c;
    bool valid;
};

struct UtcTime
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool validDate;
    bool validTime;
    bool fullyResolved;
};

struct TimeReference
{
    UtcTime utc;
    uint64_t local_us;
    bool valid;
    uint32_t update_count;
};

// ---------------- DRIVER INSTANCES ----------------
HYT939 hyt;
TSIC306 tsic(TSIC_DATA_PIN);
ADIS16407 adis;
HardwareSerial GNSS(2);

// ---------------- QUEUES ----------------
QueueHandle_t hytQueue = nullptr;
QueueHandle_t tsicQueue = nullptr;
QueueHandle_t adisQueue = nullptr;

// ---------------- SHARED TIME REFERENCE ----------------
TimeReference g_timeRef{};
SemaphoreHandle_t timeRefMutex = nullptr;

uint8_t navPvtPayload[NAV_PVT_PAYLOAD_LEN];

// ---------------- GNSS / UTC HELPERS ----------------
uint16_t readU16LE(const uint8_t *buffer, uint16_t index)
{
    return static_cast<uint16_t>(buffer[index]) |
           (static_cast<uint16_t>(buffer[index + 1]) << 8);
}

bool parseNavPvtUtc(const uint8_t *p, UtcTime &utc)
{
    // UBX-NAV-PVT payload offsets:
    // 0: iTOW, 4: year, 6: month, 7: day, 8: hour, 9: min, 10: sec, 11: valid
    utc.year = readU16LE(p, 4);
    utc.month = p[6];
    utc.day = p[7];
    utc.hour = p[8];
    utc.minute = p[9];
    utc.second = p[10];

    const uint8_t valid = p[11];
    utc.validDate = valid & 0x01;
    utc.validTime = valid & 0x02;
    utc.fullyResolved = valid & 0x04;

    return utc.validDate && utc.validTime;
}

void updateTimeReference(const UtcTime &utc)
{
    if (timeRefMutex == nullptr)
        return;

    const uint64_t local_us = esp_timer_get_time();

    if (xSemaphoreTake(timeRefMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_timeRef.utc = utc;
        g_timeRef.local_us = local_us;
        g_timeRef.valid = true;
        g_timeRef.update_count++;
        xSemaphoreGive(timeRefMutex);
    }
}

bool getTimeReference(TimeReference &copy)
{
    if (timeRefMutex == nullptr)
        return false;

    bool ok = false;

    if (xSemaphoreTake(timeRefMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        copy = g_timeRef;
        ok = g_timeRef.valid;
        xSemaphoreGive(timeRefMutex);
    }

    return ok;
}

void printUtcReference()
{
    TimeReference ref{};

    if (getTimeReference(ref))
    {
        Serial.printf(
            "[GNSS] UTC=%04u-%02u-%02u %02u:%02u:%02u | local_us=%llu | updates=%lu\n",
            ref.utc.year,
            ref.utc.month,
            ref.utc.day,
            ref.utc.hour,
            ref.utc.minute,
            ref.utc.second,
            static_cast<unsigned long long>(ref.local_us),
            static_cast<unsigned long>(ref.update_count));
    }
    else
    {
        Serial.println("[GNSS] UTC not valid yet");
    }
}

bool readUbxNavPvt()
{
    static enum
    {
        WAIT_SYNC_1,
        WAIT_SYNC_2,
        READ_CLASS,
        READ_ID,
        READ_LEN_1,
        READ_LEN_2,
        READ_PAYLOAD,
        READ_CK_A,
        READ_CK_B
    } state = WAIT_SYNC_1;

    static uint8_t msgClass = 0;
    static uint8_t msgId = 0;
    static uint16_t payloadLen = 0;
    static uint16_t payloadIndex = 0;
    static uint8_t ckA = 0;
    static uint8_t ckB = 0;
    static uint8_t receivedCkA = 0;

    while (GNSS.available())
    {
        const uint8_t b = GNSS.read();

        switch (state)
        {
        case WAIT_SYNC_1:
            if (b == UBX_SYNC_1)
                state = WAIT_SYNC_2;
            break;

        case WAIT_SYNC_2:
            if (b == UBX_SYNC_2)
            {
                ckA = 0;
                ckB = 0;
                state = READ_CLASS;
            }
            else
            {
                state = WAIT_SYNC_1;
            }
            break;

        case READ_CLASS:
            msgClass = b;
            ckA += b;
            ckB += ckA;
            state = READ_ID;
            break;

        case READ_ID:
            msgId = b;
            ckA += b;
            ckB += ckA;
            state = READ_LEN_1;
            break;

        case READ_LEN_1:
            payloadLen = b;
            ckA += b;
            ckB += ckA;
            state = READ_LEN_2;
            break;

        case READ_LEN_2:
            payloadLen |= static_cast<uint16_t>(b) << 8;
            ckA += b;
            ckB += ckA;
            payloadIndex = 0;

            if (payloadLen > NAV_PVT_PAYLOAD_LEN)
            {
                state = WAIT_SYNC_1;
            }
            else if (payloadLen == 0)
            {
                state = READ_CK_A;
            }
            else
            {
                state = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            navPvtPayload[payloadIndex++] = b;
            ckA += b;
            ckB += ckA;

            if (payloadIndex >= payloadLen)
                state = READ_CK_A;
            break;

        case READ_CK_A:
            receivedCkA = b;
            state = READ_CK_B;
            break;

        case READ_CK_B:
        {
            const uint8_t receivedCkB = b;
            const bool checksumOk = (receivedCkA == ckA) && (receivedCkB == ckB);

            if (checksumOk &&
                msgClass == UBX_CLASS_NAV &&
                msgId == UBX_ID_NAV_PVT &&
                payloadLen == NAV_PVT_PAYLOAD_LEN)
            {
                state = WAIT_SYNC_1;
                return true;
            }

            state = WAIT_SYNC_1;
            break;
        }
        }
    }

    return false;
}

// ---------------- TASKS ----------------
void taskGNSS(void *pvParameters)
{
    while (true)
    {
        if (readUbxNavPvt())
        {
            UtcTime utc{};

            if (parseNavPvtUtc(navPvtPayload, utc))
            {
                updateTimeReference(utc);
                printUtcReference();
            }
            else
            {
                Serial.println("[GNSS] NAV-PVT received, but UTC not valid yet");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void taskHYT(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(HYT_PERIOD_MS);
    const TickType_t conversionDelay = pdMS_TO_TICKS(100);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        HYTSample sample{};

        bool trig_ok = hyt.trigger();
        vTaskDelay(conversionDelay);

        sample.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sample.valid = trig_ok && hyt.readResult(sample.status,
                                                 sample.humidity_percent,
                                                 sample.temperature_c);

        if (hytQueue != nullptr)
        {
            BaseType_t queued = xQueueSend(hytQueue, &sample, 0);
            if (queued != pdPASS)
            {
                Serial.printf("[HYT939] t=%lu ms | QUEUE FULL\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void taskTSIC(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(TSIC_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        TSICSample sample{};

        sample.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sample.valid = tsic.readTempC(sample.temperature_c,
                                      sample.raw11,
                                      sample.tStrobe_us);

        if (tsicQueue != nullptr)
        {
            BaseType_t queued = xQueueSend(tsicQueue, &sample, 0);
            if (queued != pdPASS)
            {
                Serial.printf("[TSIC306] t=%lu ms | QUEUE FULL\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void taskADIS(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(ADIS_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        ADISSample sample{};

        sample.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sample.temperature_c = adis.readTempC();
        sample.valid = true;

        if (adisQueue != nullptr)
        {
            BaseType_t queued = xQueueSend(adisQueue, &sample, 0);
            if (queued != pdPASS)
            {
                Serial.printf("[ADIS16407] t=%lu ms | QUEUE FULL\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void taskLoggerHYT(void *pvParameters)
{
    HYTSample sample{};

    while (true)
    {
        if (xQueueReceive(hytQueue, &sample, portMAX_DELAY) == pdPASS)
        {
            if (sample.valid)
            {
                Serial.printf(
                    "[HYT939] t=%lu ms | status=%u | RH=%.2f %% | T=%.2f C\n",
                    static_cast<unsigned long>(sample.timestamp_ms),
                    sample.status,
                    sample.humidity_percent,
                    sample.temperature_c);
            }
            else
            {
                Serial.printf("[HYT939] t=%lu ms | ERROR\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }
    }
}

void taskLoggerTSIC(void *pvParameters)
{
    TSICSample sample{};

    while (true)
    {
        if (xQueueReceive(tsicQueue, &sample, portMAX_DELAY) == pdPASS)
        {
            if (sample.valid)
            {
                Serial.printf(
                    "[TSIC306] t=%lu ms | raw11=%u | Tstrobe=%lu us | T=%.2f C\n",
                    static_cast<unsigned long>(sample.timestamp_ms),
                    sample.raw11,
                    static_cast<unsigned long>(sample.tStrobe_us),
                    sample.temperature_c);
            }
            else
            {
                Serial.printf("[TSIC306] t=%lu ms | ERROR\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }
    }
}

void taskLoggerADIS(void *pvParameters)
{
    ADISSample sample{};

    while (true)
    {
        if (xQueueReceive(adisQueue, &sample, portMAX_DELAY) == pdPASS)
        {
            if (sample.valid)
            {
                Serial.printf(
                    "[ADIS16407] t=%lu ms | Temp=%.2f C\n",
                    static_cast<unsigned long>(sample.timestamp_ms),
                    sample.temperature_c);
            }
            else
            {
                Serial.printf("[ADIS16407] t=%lu ms | ERROR\n",
                              static_cast<unsigned long>(sample.timestamp_ms));
            }
        }
    }
}

void taskHeartbeat(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS);

    while (true)
    {
        const uint32_t timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        TimeReference ref{};
        const bool utcReady = getTimeReference(ref);

        Serial.printf("[Heartbeat] t=%lu ms | system alive | UTC ready=%u\n",
                      static_cast<unsigned long>(timestamp_ms),
                      utcReady);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// ---------------- SETUP ----------------
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== HYT939 + TSIC306 + ADIS16407 + GNSS UTC FreeRTOS Test ===");

    Wire.begin();
    SPI.begin();
    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

    timeRefMutex = xSemaphoreCreateMutex();
    if (timeRefMutex == nullptr)
    {
        Serial.println("Failed to create UTC time-reference mutex!");
        while (true)
        {
            delay(1000);
        }
    }

    if (!hyt.begin(Wire, HYT_I2C_ADDRESS))
    {
        Serial.println("HYT939 init failed!");
        while (true)
        {
            delay(1000);
        }
    }

    if (!adis.begin(SPI, ADIS_CS_PIN))
    {
        Serial.println("ADIS16407 init failed!");
        while (true)
        {
            delay(1000);
        }
    }

    adis.primePipeline();
    tsic.begin();

    hytQueue = xQueueCreate(HYT_QUEUE_LENGTH, sizeof(HYTSample));
    if (hytQueue == nullptr)
    {
        Serial.println("Failed to create HYT queue!");
        while (true)
        {
            delay(1000);
        }
    }

    tsicQueue = xQueueCreate(TSIC_QUEUE_LENGTH, sizeof(TSICSample));
    if (tsicQueue == nullptr)
    {
        Serial.println("Failed to create TSIC queue!");
        while (true)
        {
            delay(1000);
        }
    }

    adisQueue = xQueueCreate(ADIS_QUEUE_LENGTH, sizeof(ADISSample));
    if (adisQueue == nullptr)
    {
        Serial.println("Failed to create ADIS queue!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("HYT939 initialized.");
    Serial.println("TSIC306 initialized.");
    Serial.println("ADIS16407 initialized.");
    Serial.println("GNSS UART initialized.");
    Serial.println("Queues created.");

    xTaskCreatePinnedToCore(
        taskGNSS,
        "GNSS_Task",
        GNSS_STACK_SIZE,
        nullptr,
        GNSS_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskHYT,
        "HYT_Task",
        HYT_STACK_SIZE,
        nullptr,
        SENSOR_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskTSIC,
        "TSIC_Task",
        TSIC_STACK_SIZE,
        nullptr,
        SENSOR_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskADIS,
        "ADIS_Task",
        ADIS_STACK_SIZE,
        nullptr,
        ADIS_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskLoggerHYT,
        "Logger_HYT",
        LOGGER_STACK_SIZE,
        nullptr,
        LOGGER_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskLoggerTSIC,
        "Logger_TSIC",
        LOGGER_STACK_SIZE,
        nullptr,
        LOGGER_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskLoggerADIS,
        "Logger_ADIS",
        LOGGER_STACK_SIZE,
        nullptr,
        LOGGER_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskHeartbeat,
        "Heartbeat_Task",
        HEARTBEAT_STACK_SIZE,
        nullptr,
        HEARTBEAT_PRIORITY,
        nullptr,
        CORE);
}

// ---------------- LOOP ----------------
void loop()
{
    vTaskDelete(nullptr);  // kill Arduino loop task
}
