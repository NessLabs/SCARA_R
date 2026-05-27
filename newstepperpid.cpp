#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "SimpleStepper.h"
#include "SimpleTimer.h"
#include "AS5600.h"
#include "SimplePID.h"

static const char* TAG = "StepperPID";

#define STEP_PIN    5
#define DIR_PIN     18
#define EN_PIN      33
#define SDA_PIN     21
#define SCL_PIN     22

#define CONTROL_HZ  500
#define CONTROL_US  (1000000 / CONTROL_HZ)
#define DT          (1.0f / CONTROL_HZ)

#define TOLERANCE   2.0f   // degrees — tune as needed
#define DEADBAND    0.05f  // PID output threshold to trigger a step

SimpleStepper stepper;
SimpleTimer   controlTimer;
AS5600        encoder;
SimplePID     pid;

volatile float   g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;
volatile float   g_target    = 0.0f;

// encoder task — reads I2C at 50Hz, never blocks control loop
void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// control loop — PID decides direction and whether to step
// output > deadband  → step forward
// output < -deadband → step backward
// |output| < deadband → hold (no step, no vibration)
void controlLoop(void* arg)
{
    float error  = g_target - g_encAngle;

    // within tolerance — disable motor, reset PID
    if (fabsf(error) <= TOLERANCE)
    {
        stepper.disable();
        return;
    }

    float output = pid.calc(error);

    if (output > DEADBAND)
    {
        stepper.enable();
        // set direction and pulse one step
        gpio_set_level((gpio_num_t)DIR_PIN, 1);
        gpio_set_level((gpio_num_t)STEP_PIN, 1);
        ets_delay_us(5);
        gpio_set_level((gpio_num_t)STEP_PIN, 0);
    }
    else if (output < -DEADBAND)
    {
        stepper.enable();
        gpio_set_level((gpio_num_t)DIR_PIN, 0);
        gpio_set_level((gpio_num_t)STEP_PIN, 1);
        ets_delay_us(5);
        gpio_set_level((gpio_num_t)STEP_PIN, 0);
    }
    else
    {
        stepper.disable();  // in deadband — cut power
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
    g_target = deg;
    pid.reset();

    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_target - g_encAngle) > TOLERANCE)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(5000))
        {
            ESP_LOGW(TAG, "timeout at %.2f deg", g_encAngle);
            break;
        }
        ESP_LOGI(TAG, "target: %.2f | encoder: %.2f | error: %.2f | magnet: %s",
                 g_target, g_encAngle,
                 g_target - g_encAngle,
                 magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "done — encoder: %.2f | magnet: %s",
             g_encAngle, magnetStatus());
}

extern "C" void app_main()
{
    // stepper just for enable/disable and pin setup
    stepper.setup(STEP_PIN, DIR_PIN, EN_PIN, 200, 1, 3.0f);

    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found — check wiring and magnet");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready — magnet: %s", magnetStatus());
    }

    // PID output is a step trigger, not a position
    // tune Kp until it moves decisively, add Kd to reduce overshoot
    float gains[3] = { 0.05f, 0.0f, 0.005f };
    pid.setup(gains, DT, -1.0f, 1.0f);

    xTaskCreate(encoderTask, "encoder", 2048, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));

    g_target = g_encAngle;  // hold current position

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Stepper PID Test ===");
    moveTo(45.0f);
    moveTo(0.0f);
    ESP_LOGI(TAG, "=== Done ===");

    while (true)
    {
        ESP_LOGI(TAG, "target: %.2f | encoder: %.2f | diff: %.2f | magnet: %s",
                 g_target, g_encAngle,
                 g_target - g_encAngle,
                 magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}