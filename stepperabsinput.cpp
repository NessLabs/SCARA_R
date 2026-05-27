#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

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

    // Configure stdin to be non-blocking so it doesn't freeze the while loop
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    ESP_LOGI(TAG, "=== Stepper Serial Control Ready ===");
    ESP_LOGI(TAG, "Type a target angle and press Enter:");

    char rx_buf[32];
    int rx_idx = 0;
    uint32_t last_print = 0;

    while (true)
    {
        // 1. Check for incoming serial data
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                if (rx_idx > 0) {
                    rx_buf[rx_idx] = '\0';            // Null-terminate the string
                    float target_deg = atof(rx_buf);  // Convert to float
                    rx_idx = 0;                       // Reset buffer index for next input
                    
                    ESP_LOGI(TAG, "Input received: %.2f", target_deg);
                    moveTo(target_deg);               // Move the motor
                }
            } else if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;                 // Store character
            }
        }

        // 2. Print status every 1000 ms
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        if (current_time - last_print > 1000) {
            ESP_LOGI(TAG, "stepper: %.2f | encoder: %.2f | diff: %.2f | magnet: %s",
                     stepper.getAngle(), g_encAngle,
                     stepper.getAngle() - g_encAngle,
                     magnetStatus());
            last_print = current_time;
        }

        // 3. Small delay to feed the task watchdog and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}