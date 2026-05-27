#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "SimpleStepper.h"
#include "SimpleTimer.h"
#include "AS5600.h"

static const char* TAG = "StepperTest";

// Stepper 1 Pins
#define STEP_1_PIN  5 
#define DIR_1_PIN   18 

// Stepper 2 Pins
#define STEP_2_PIN  19
#define DIR_2_PIN   23

// Shared Enable and I2C
#define EN_PIN      33 
#define SDA_PIN     21
#define SCL_PIN     22

#define CONTROL_HZ  100
#define CONTROL_US  (1000000 / CONTROL_HZ)

SimpleStepper stepper1;
SimpleStepper stepper2;
SimpleTimer   controlTimer;
AS5600        encoder;

volatile float g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;

void controlLoop(void* arg)
{
    // Update both steppers continuously in the background
    stepper1.update();  
    stepper2.update();
}

void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(20));  // read at 50Hz
    }
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

// Moves simultaneously (Both motors start and run at the same time)
void moveSimultaneously(float deg1, float deg2)
{
    // --- OPTIONAL: Uncomment to enable drivers only during movement ---
    // gpio_set_level((gpio_num_t)EN_PIN, 0); 
    
    ESP_LOGI(TAG, "→ Moving Stepper1 to: %.2f deg | Stepper2 to: %.2f deg", deg1, deg2);
    
    // Set both targets before entering the waiting loop
    stepper1.setTarget(deg1);
    stepper2.setTarget(deg2);
    
    // Block until BOTH steppers have reached their targets
    // The loop continues as long as EITHER stepper is not at its target
    while (!stepper1.atTarget() || !stepper2.atTarget())
    {
        ESP_LOGI(TAG, "Moving... S1: %.2f | S2: %.2f | enc: %.2f", 
                 stepper1.getAngle(), stepper2.getAngle(), g_encAngle);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    ESP_LOGI(TAG, "Both steppers arrived. Simultaneous movement complete!");
    
    // --- OPTIONAL: Uncomment to disable drivers while idle (saves power/heat) ---
    // gpio_set_level((gpio_num_t)EN_PIN, 1);
}

extern "C" void app_main()
{
    // --- MANUAL ENABLE CONTROL ---
    // Configure the real EN_PIN (33) as an output
    gpio_reset_pin((gpio_num_t)EN_PIN);
    gpio_set_direction((gpio_num_t)EN_PIN, GPIO_MODE_OUTPUT);
    
    // Pull it LOW to enable the drivers immediately.
    // By keeping it LOW continuously, the motors retain their holding torque.
    gpio_set_level((gpio_num_t)EN_PIN, 0); 
    // -----------------------------

    // Initialize Steppers with dummy EN pins (27 and 26) 
    // so the SimpleStepper library doesn't try to auto-disable our manual pin.
    stepper1.setup(STEP_1_PIN, DIR_1_PIN, 27, 200, 1, 3.0f);
    stepper2.setup(STEP_2_PIN, DIR_2_PIN, 26, 200, 1, 3.0f);

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

    // Configure standard input to not block the main loop
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    ESP_LOGI(TAG, "=== Simultaneous Stepper Control Ready ===");
    ESP_LOGI(TAG, "Type target angles format 'S1,S2' (e.g., '180,90') and press Enter:");

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
                    rx_buf[rx_idx] = '\0';            
                    
                    float target1 = 0.0f, target2 = 0.0f;
                    
                    // Parse 'x,y' format
                    if (sscanf(rx_buf, "%f,%f", &target1, &target2) == 2) {
                        ESP_LOGI(TAG, "Input received: S1=%.2f, S2=%.2f", target1, target2);
                        moveSimultaneously(target1, target2); // Changed to simultaneous
                    } else {
                        ESP_LOGW(TAG, "Invalid format! Please use 'number,number' (e.g. 180,90)");
                    }
                    
                    rx_idx = 0;                       
                }
            } else if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;                 
            }
        }

        // 2. Print status every 1000 ms
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        if (current_time - last_print > 1000) {
            ESP_LOGI(TAG, "S1: %.2f | S2: %.2f | enc: %.2f | diff(S1): %.2f",
                     stepper1.getAngle(), stepper2.getAngle(), g_encAngle,
                     stepper1.getAngle() - g_encAngle);
            last_print = current_time;
        }

        // 3. Small delay to feed the task watchdog
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}