#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"

static const char* TAG = "DCTest";

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
    motor2.update();
}

// Dedicated FreeRTOS Task to read and parse Serial Input
void serialInputTask(void* arg)
{
    char line[64];
    int pos = 0;

    ESP_LOGI(TAG, "Serial Input Ready. Send targets in format: x,y (e.g., 90.0,-45.5)");

    while (true)
    {
        int c = fgetc(stdin); // Read character from standard input

        // Check if character is valid (EOF usually means no data available)
        if (c != EOF && c != 255) 
        {
            // If newline or carriage return, parse the line
            if (c == '\n' || c == '\r') 
            {
                if (pos > 0) 
                {
                    line[pos] = '\0'; // Null-terminate string
                    float m1_target = 0.0f;
                    float m2_target = 0.0f;

                    // Parse the format "x,y"
                    if (sscanf(line, "%f,%f", &m1_target, &m2_target) == 2) 
                    {
                        ESP_LOGI(TAG, "==> Command Received: M1=%.2f, M2=%.2f", m1_target, m2_target);
                        motor1.setTarget(m1_target);
                        motor2.setTarget(m2_target);
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "Invalid format! Please use 'x,y' (e.g., 90,45)");
                    }
                    pos = 0; // Reset buffer position for next line
                }
            } 
            // Add character to buffer if there's room
            else if (pos < sizeof(line) - 1) 
            {
                line[pos++] = (char)c;
            }
        }
        
        // Brief delay to yield to the FreeRTOS scheduler
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

extern "C" void app_main()
{
    // 1. Setup PWM Timer Configuration
    TimerConfig bdcTimer;
    bdcTimer.frequency = 20000; // 20kHz PWM frequency

    // 2. Setup PID Gains and Encoder Math
    float gains[3]      = { 1.0f, 0.1f, 0.05f };  // Kp, Ki, Kd
    uint8_t encPins1[2] = { BDC1_ENC_A, BDC1_ENC_B };
    uint8_t encPins2[2] = { BDC2_ENC_A, BDC2_ENC_B };
    
    // Calculate degrees per encoder edge
    float degPerEdge    = 360.0f / (1026.0f  * 3.0f); 

    // 3. Initialize the Motors
    motor1.setup(BDC1_PWM_PIN, BDC1_IN1_PIN, BDC1_IN2_PIN, encPins1, gains,
                 CONTROL_DT, degPerEdge, &bdcTimer, 0);

    motor2.setup(BDC2_PWM_PIN, BDC2_IN1_PIN, BDC2_IN2_PIN, encPins2, gains,
                 CONTROL_DT, degPerEdge, &bdcTimer, 1);

    // 4. Start the high-priority control loop timer
    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    
    // Brief delay to let sensors stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Starting Dual DC Motor Control ===");

    // 5. Start the serial input listening task
    // Arguments: Function, Name, Stack Size, Param, Priority, Handle
    xTaskCreate(serialInputTask, "serial_input", 4096, NULL, 1, NULL);

    // Loop forever printing the current status
    while (true)
    {
        ESP_LOGI(TAG, "M1 pos: %.2f° (tgt: %.2f°) | M2 pos: %.2f° (tgt: %.2f°)",
                 motor1.getAngle(), motor1.getTarget(),
                 motor2.getAngle(), motor2.getTarget());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}