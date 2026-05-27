#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleStepper.h"
#include "SimpleTimer.h"
#include "AS5600.h"
#include "math.h"

static const char* TAG = "ClosedLoopStepper";

#define STEP_PIN     5  
#define DIR_PIN      18 
#define EN_PIN       33 
#define SDA_PIN      21
#define SCL_PIN      22

#define CONTROL_HZ   100
#define CONTROL_US   (1000000 / CONTROL_HZ)

SimpleStepper stepper;
SimpleTimer   controlTimer;
AS5600        encoder;

// State Variables
volatile float g_targetAngle = 0.0f; 
volatile float g_encAngle    = 0.0f;
volatile uint8_t g_encStatus = 0;

// The "King" Logic: Correcting stepper position based on encoder feedback
void controlLoop(void* arg)
{
    // 1. Calculate Error (Where we want to be - where we actually are)
    float error = g_targetAngle - g_encAngle;

    // 2. Deadzone: Don't jitter if we are within 0.5 degrees
    if (fabs(error) < 0.5f) {
        // We are at the target, stop pulsing
        return; 
    }

    /* 3. Feed the error into the stepper. 
       Instead of absolute position, we tell the stepper to move 
       relatively to close the gap detected by the encoder.
    */
    stepper.setTarget(stepper.getAngle() + error);
    stepper.update(); 
}

void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling for the "King"
    }
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG";
    return "NOT DETECTED";
}

void closedLoopMoveTo(float deg)
{
    ESP_LOGI(TAG, "Setting Absolute Target: %.2f deg", deg);
    g_targetAngle = deg;

    // Wait until the ENCODER (not the stepper software) reports we arrived
    while (fabs(g_targetAngle - g_encAngle) > 1.0f) 
    {
        ESP_LOGI(TAG, "Target: %.2f | Actual: %.2f | Magnet: %s",
                 g_targetAngle, g_encAngle, magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Target Reached!");
}

extern "C" void app_main()
{
    // Standard Setup
    stepper.setup(STEP_PIN, DIR_PIN, EN_PIN, 200, 1, 3.0f);
    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);

    if (!encoder.isConnected()) {
        ESP_LOGE(TAG, "CRITICAL: Encoder not found. System halted.");
        return;
    }

    encoder.setZero();
    
    // Start the Encoder task first to get a valid reading
    xTaskCreate(encoderTask, "encoder", 2048, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Start the Control loop
    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);

    // Test the "King"
    closedLoopMoveTo(30.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    closedLoopMoveTo(0.0f);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}