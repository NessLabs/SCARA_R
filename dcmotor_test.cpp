#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"

static const char* TAG = "DCTest";

// --- Pin Definitions ---
#define BDC_PWM_PIN  13
#define BDC_IN1_PIN  14  // Logic Input 1 14
#define BDC_IN2_PIN  27  // Logic Input 2 (Added for 3-pin bridge)
#define BDC_ENC_A    36 
#define BDC_ENC_B    39 

// --- Control Timing ---
#define CONTROL_HZ   100
#define CONTROL_US   (1000000 / CONTROL_HZ)
#define CONTROL_DT   (1.0f / CONTROL_HZ)

SimpleBDC   motor;
SimpleTimer controlTimer;

// This function is called by the timer every 10ms (100Hz)
void controlLoop(void* arg)
{
    motor.update();
}

void moveTo(float deg)
{
    ESP_LOGI(TAG, "→ Moving to: %.2f deg", deg);
    motor.setTarget(deg);
    
    // Wait for 2 seconds to let the motor reach position
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Currently at: %.2f deg", motor.getAngle());
}

extern "C" void app_main()
{
    // 1. Setup PWM Timer Configuration
    TimerConfig bdcTimer;
    bdcTimer.frequency = 20000; // 20kHz PWM frequency

    // 2. Setup PID Gains and Encoder Math
    float gains[3]     = { 1.0f, 0.1f, 0.05f };  // Kp, Ki, Kd
    uint8_t encPins[2] = { BDC_ENC_A, BDC_ENC_B };
    
    // Calculate degrees per encoder edge
    // Based on your values: 1026 pulses/rev * 4 (quadrature) * gear ratio
    float degPerEdge   = 360.0f / (1026.0f  * 3.0f); 

    // 3. Initialize the Motor with the new 3-pin setup
    // Arguments: PWM, IN1, IN2, Encoder Pins, PID Gains, DT, Scale, Timer, Channel
    motor.setup(BDC_PWM_PIN, BDC_IN1_PIN, BDC_IN2_PIN, encPins, gains,
                CONTROL_DT, degPerEdge, &bdcTimer, 0);

    // 4. Start the high-priority control loop timer
    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    
    // Brief delay to let sensors stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Starting DC Motor Test (3-Pin Mode) ===");
    
    moveTo(90.0f);
    
    ESP_LOGI(TAG, "=== Sequence Done ===");

    // Loop forever printing the current status
    while (true)
    {
        ESP_LOGI(TAG, "pos: %.2f° | target: %.2f°",
                 motor.getAngle(), motor.getTarget());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}