#include <Arduino.h>
#include <Wire.h>
#include "hyt939.h"

// ---------------- CONFIG ----------------
constexpr uint32_t HYT_PERIOD_MS = 120;   // ~8.33 Hz, leaves time for sensor conversion
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
constexpr uint32_t STACK_SIZE = 4096;
constexpr uint32_t HEARTBEAT_STACK_SIZE = 2048;
constexpr uint32_t LOGGER_STACK_SIZE = 4096;
constexpr UBaseType_t SENSOR_PRIORITY = 2;
constexpr UBaseType_t LOGGER_PRIORITY = 1;
constexpr UBaseType_t HEARTBEAT_PRIORITY = 1;
constexpr BaseType_t CORE = 1;
constexpr uint8_t HYT_I2C_ADDRESS = 0x28;
constexpr size_t HYT_QUEUE_LENGTH = 10;

// ---------------- DATA TYPES ----------------
struct HYTSample
{
    uint32_t timestamp_ms;
    uint8_t status;
    float humidity_percent;
    float temperature_c;
    bool valid;
};

// ---------------- DRIVER INSTANCE ----------------
HYT939 hyt;
QueueHandle_t hytQueue = nullptr;

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

void taskLogger(void *pvParameters)
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
    Serial.println("\n=== HYT939 FreeRTOS Queue Test ===");

    Wire.begin();

    if (!hyt.begin(Wire, HYT_I2C_ADDRESS))
    {
        Serial.println("HYT939 init failed!");
        while (true)
        {
            delay(1000);
        }
    }

    hytQueue = xQueueCreate(HYT_QUEUE_LENGTH, sizeof(HYTSample));
    if (hytQueue == nullptr)
    {
        Serial.println("Failed to create HYT queue!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("HYT939 initialized.");
    Serial.println("Queue created.");

    xTaskCreatePinnedToCore(
        taskHYT,
        "HYT_Task",
        STACK_SIZE,
        nullptr,
        SENSOR_PRIORITY,
        nullptr,
        CORE);

    xTaskCreatePinnedToCore(
        taskLogger,
        "Logger_Task",
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