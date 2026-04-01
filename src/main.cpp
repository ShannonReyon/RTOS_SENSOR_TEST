#include <Arduino.h>

TaskHandle_t task1Handle = nullptr;
TaskHandle_t task2Handle = nullptr;

// -----------------------------
// Task 1
// -----------------------------
void taskSensorA(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(1000); // 1 second
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.printf("[Task A] Running on core %d, tick = %lu ms\n",
                      xPortGetCoreID(),
                      (unsigned long)millis());

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

// -----------------------------
// Task 2
// -----------------------------
void taskSensorB(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(500); // 500 ms
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        Serial.printf("[Task B] Running on core %d, tick = %lu ms\n",
                      xPortGetCoreID(),
                      (unsigned long)millis());

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting FreeRTOS task test...");

    xTaskCreatePinnedToCore(
        taskSensorA,     // task function
        "TaskSensorA",   // name
        4096,            // stack size
        nullptr,         // parameter
        1,               // priority
        &task1Handle,    // handle
        1                // core
    );

    xTaskCreatePinnedToCore(
        taskSensorB,
        "TaskSensorB",
        4096,
        nullptr,
        1,
        &task2Handle,
        1
    );

    Serial.println("Tasks created.");
}

void loop()
{
    // Intentionally unused.
    vTaskDelete(nullptr); // delete Arduino loop task
}