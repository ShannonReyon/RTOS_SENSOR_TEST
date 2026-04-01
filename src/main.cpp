#include <Arduino.h>
#include <Wire.h>
#include "hyt939.h"

// ---------------- CONFIG ----------------
constexpr uint32_t HYT_PERIOD_MS = 100;   // 10 Hz
constexpr uint32_t STACK_SIZE    = 4096;
constexpr UBaseType_t PRIORITY   = 1;
constexpr BaseType_t CORE        = 1;

// ---------------- DRIVER INSTANCE ----------------
HYT939 hyt;

// ---------------- TASK ----------------
void taskHYT(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(100);   // 10 Hz
    const TickType_t conversionDelay = pdMS_TO_TICKS(100);

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        uint8_t status = 0;
        float humidity = 0.0f;
        float temperature = 0.0f;

        // -------- Phase 1: trigger --------
        bool trig_ok = hyt.trigger();

        // Wait for sensor conversion (non-blocking!)
        vTaskDelay(conversionDelay);

        // -------- Phase 2: read --------
        bool read_ok = hyt.readResult(status, humidity, temperature);

        uint32_t t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (trig_ok && read_ok)
        {
            Serial.printf(
                "[HYT939] t=%lu ms | RH=%.2f %% | T=%.2f C\n",
                t_ms,
                humidity,
                temperature
            );
        }
        else
        {
            Serial.printf("[HYT939] t=%lu ms | ERROR\n", t_ms);
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// ---------------- SETUP ----------------
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== HYT939 FreeRTOS Test ===");

    Wire.begin();

    if (!hyt.begin(Wire, 0x28))
    {
        Serial.println("HYT939 init failed!");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("HYT939 initialized.");

    xTaskCreatePinnedToCore(
        taskHYT,
        "HYT_Task",
        STACK_SIZE,
        nullptr,
        PRIORITY,
        nullptr,
        CORE
    );
}

// ---------------- LOOP ----------------
void loop()
{
    vTaskDelete(nullptr);  // kill Arduino loop task
}