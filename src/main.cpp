#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
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
constexpr uint32_t LOGGER_STACK_SIZE = 4096;
constexpr uint32_t HEARTBEAT_STACK_SIZE = 2048;

constexpr UBaseType_t SENSOR_PRIORITY = 2;
constexpr UBaseType_t ADIS_PRIORITY = 3;
constexpr UBaseType_t LOGGER_PRIORITY = 1;
constexpr UBaseType_t HEARTBEAT_PRIORITY = 1;

constexpr BaseType_t CORE = 1;

constexpr uint8_t HYT_I2C_ADDRESS = 0x28;
constexpr size_t HYT_QUEUE_LENGTH = 10;
constexpr size_t TSIC_QUEUE_LENGTH = 10;
// Set this to your actual TSIC data pin
constexpr int TSIC_DATA_PIN = 4;
// Set this to your actual ADIS chip-select pin
constexpr int ADIS_CS_PIN = 5;
constexpr size_t ADIS_QUEUE_LENGTH = 10;

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

// ---------------- DRIVER INSTANCES ----------------
HYT939 hyt;
TSIC306 tsic(TSIC_DATA_PIN);
ADIS16407 adis;

// ---------------- QUEUES ----------------
QueueHandle_t hytQueue = nullptr;
QueueHandle_t tsicQueue = nullptr;
QueueHandle_t adisQueue = nullptr;

// ---------------- TASKS ----------------
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
        Serial.printf("[Heartbeat] t=%lu ms | system alive\n",
                      static_cast<unsigned long>(timestamp_ms));
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// ---------------- SETUP ----------------
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== HYT939 + TSIC306 FreeRTOS Queue Test ===");

    Wire.begin();
    SPI.begin();

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
    Serial.println("Queues created.");

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