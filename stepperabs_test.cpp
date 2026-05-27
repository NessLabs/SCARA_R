#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleStepper.h"
#include "SimpleTimer.h"
#include "AS5600.h"

static const char* TAG = "StepperTest";

#define STEP_PIN    5 //25  
#define DIR_PIN     18 //26
#define EN_PIN      33 //27
#define SDA_PIN     21
#define SCL_PIN     22

#define CONTROL_HZ  100
#define CONTROL_US  (1000000 / CONTROL_HZ)

SimpleStepper stepper;
SimpleTimer   controlTimer;
AS5600        encoder;

// cached encoder value — updated by encoderTask, read by anyone
volatile float g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;

void controlLoop(void* arg)
{
    stepper.update();  // no I2C here
}

void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(20));  // read at 50Hz — fast enough, slow enough
    }
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

void moveTo(float deg)
{
    ESP_LOGI(TAG, "→ %.2f deg", deg);
    stepper.setTarget(deg);
    while (!stepper.atTarget())
    {
        ESP_LOGI(TAG, "stepper: %.2f | encoder: %.2f | magnet: %s",
                 stepper.getAngle(), g_encAngle, magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "done — stepper: %.2f | encoder: %.2f | diff: %.2f | magnet: %s",
             stepper.getAngle(), g_encAngle,
             stepper.getAngle() - g_encAngle,
             magnetStatus());
}

extern "C" void app_main()
{
    stepper.setup(STEP_PIN, DIR_PIN, EN_PIN, 200, 1, 3.0f);

    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found — check wiring and magnet");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready — magnet: %s", magnetStatus());
    }

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);

    xTaskCreate(encoderTask, "encoder", 2048, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Stepper Test ===");
    moveTo(45.0f);
    moveTo(0.0f);
    ESP_LOGI(TAG, "=== Done ===");

    while (true)
    {
        ESP_LOGI(TAG, "stepper: %.2f | encoder: %.2f | diff: %.2f | magnet: %s",
                 stepper.getAngle(), g_encAngle,
                 stepper.getAngle() - g_encAngle,
                 magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}