#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"

static const char* TAG = "DCTest";

// --- Pin Definitions ---
#define LIMIT_SWITCH_PIN 4 // Motor 2 Limit Switch

// --- Conversion Constants ---
#define M2_DEG_PER_CM 76.92f

// --- Motor 1 Pin Definitions ---
#define BDC1_PWM_PIN  13
#define BDC1_IN1_PIN  14  
#define BDC1_IN2_PIN  27  
#define BDC1_ENC_A    36 
#define BDC1_ENC_B    39 

// --- Motor 2 Pin Definitions ---
#define BDC2_PWM_PIN  32
#define BDC2_IN1_PIN  25  
#define BDC2_IN2_PIN  26  
#define BDC2_ENC_A    34 
#define BDC2_ENC_B    35 

// --- Control Timing ---
#define CONTROL_HZ   100
#define CONTROL_US   (1000000 / CONTROL_HZ)
#define CONTROL_DT   (1.0f / CONTROL_HZ)

// --- Motor Instances ---
SimpleBDC   motor1;
SimpleBDC   motor2;
SimpleTimer controlTimer;

// This function is called by the timer every 10ms (100Hz)
void controlLoop(void* arg)
{
    motor1.update();

    // Read the limit switch state. (Active HIGH - reads 1 when pressed)
    if (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 1) 
    {
        // The switch is active. 
        // Only stop if the command is telling it to go further negative!
        if (motor2.getTarget() < motor2.getAngle()) 
        {
            // Cuts power and sets _running = false.
            motor2.stop();
        }
    }

    // If _running is false, your library cleanly ignores this update.
    motor2.update();
}

// Dedicated FreeRTOS Task to read and parse Serial Input
void serialInputTask(void* arg)
{
    char line[64];
    int pos = 0;

    // Updated instructions to reflect new units
    ESP_LOGI(TAG, "Serial Input Ready. Send targets as: deg,cm (e.g., 90.0,-5.5)");

    while (true)
    {
        int c = fgetc(stdin); 

        if (c != EOF && c != 255) 
        {
            if (c == '\n' || c == '\r') 
            {
                if (pos > 0) 
                {
                    line[pos] = '\0'; 
                    float m1_target_deg = 0.0f;
                    float m2_target_cm  = 0.0f;

                    // Read X as degrees, Y as cm
                    if (sscanf(line, "%f,%f", &m1_target_deg, &m2_target_cm) == 2) 
                    {
                        // Convert cm to degrees for the motor controller
                        float m2_target_deg = m2_target_cm * M2_DEG_PER_CM;

                        ESP_LOGI(TAG, "==> Command Received: M1=%.2f°, M2=%.2f cm (Internally: %.2f°)", 
                                 m1_target_deg, m2_target_cm, m2_target_deg);
                                 
                        motor1.setTarget(m1_target_deg);
                        motor2.setTarget(m2_target_deg); // Send calculated degrees to motor
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "Invalid format! Please use 'deg,cm' (e.g., 90,-5.5)");
                    }
                    pos = 0; 
                }
            } 
            else if (pos < sizeof(line) - 1) 
            {
                line[pos++] = (char)c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

extern "C" void app_main()
{
    // 0. Setup Limit Switch Hardware (Active HIGH Configuration)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;      // Disabled pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;   // Enabled pull-down
    gpio_config(&io_conf);

    // 1. Setup PWM Timer Configuration
    TimerConfig bdcTimer;
    bdcTimer.frequency = 20000;

    // 2. Setup PID Gains and Encoder Math
    float gains[3]      = { 1.0f, 0.1f, 0.05f };
    uint8_t encPins1[2] = { BDC1_ENC_A, BDC1_ENC_B };
    uint8_t encPins2[2] = { BDC2_ENC_A, BDC2_ENC_B };
    
    // We leave the base math in degrees to keep PID tolerances working correctly
    float degPerEdge1    = 360.0f / (1026.0f  * 3.0f);    
    float degPerEdge2    = 360.0f / (1026.0f  * 3.0f); 

    // 3. Initialize the Motors
    motor1.setup(BDC1_PWM_PIN, BDC1_IN1_PIN, BDC1_IN2_PIN, encPins1, gains,
                 CONTROL_DT, degPerEdge1, &bdcTimer, 0);

    motor2.setup(BDC2_PWM_PIN, BDC2_IN1_PIN, BDC2_IN2_PIN, encPins2, gains,
                 CONTROL_DT, degPerEdge2, &bdcTimer, 1);

    // 4. Start the high-priority control loop timer
    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Starting Dual DC Motor Control ===");

    // 5. Start the serial input listening task
    xTaskCreate(serialInputTask, "serial_input", 4096, NULL, 1, NULL);

    while (true)
    {
        int switchState = gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN);
        
        // Calculate cm for printing
        float m2_pos_cm = motor2.getAngle() / M2_DEG_PER_CM;
        float m2_tgt_cm = motor2.getTarget() / M2_DEG_PER_CM;
        
        ESP_LOGI(TAG, "M1: %.2f° (tgt: %.2f°) | M2: %.2f cm (tgt: %.2f cm) | SW: %d",
                 motor1.getAngle(), motor1.getTarget(),
                 m2_pos_cm, m2_tgt_cm,
                 switchState);
                 
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}