#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleStepper.h"
#include "SimpleTimer.h"

static const char* TAG = "StepperTest";


#define STEP_PIN    5
#define DIR_PIN     18
#define EN_PIN      33


#define CONTROL_HZ  100
#define CONTROL_US  (1000000 / CONTROL_HZ)

SimpleStepper stepper;
SimpleTimer   controlTimer;

void controlLoop(void* arg)
{
    stepper.update();
}

void moveTo(float deg)
{
    ESP_LOGI(TAG, "→ %.2f deg", deg);
    stepper.setTarget(deg);
    while (!stepper.atTarget())
        vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "at %.2f deg", stepper.getAngle());
}

extern "C" void app_main()
{
    stepper.setup(STEP_PIN, DIR_PIN, EN_PIN, 200, 1, 3.0f);

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Stepper Test ===");
    moveTo(1000.0f);
    

    ESP_LOGI(TAG, "=== Done ===");

    while (true)
    {
        ESP_LOGI(TAG, "pos: %.2f deg", stepper.getAngle());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}