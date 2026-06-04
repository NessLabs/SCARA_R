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

static const char* TAG = "StepperTest";

// ─── Pins ─────────────────────────────────────────────────────────────
#define J1_STEP_PIN  5
#define J1_DIR_PIN   18
#define J2_STEP_PIN  19
#define J2_DIR_PIN   23
#define EN_PIN       33   // shared enable pin for both steppers
#define SDA_PIN      21
#define SCL_PIN      22

// ─── Control timing ───────────────────────────────────────────────────
#define CONTROL_HZ   500
#define CONTROL_US   (1000000 / CONTROL_HZ)
#define DT           (1.0f / CONTROL_HZ)

#define TOLERANCE    2.0f
#define DEADBAND     0.5f

// ─── Objects ─────────────────────────────────────────────────────────
SimpleStepper motorJ1, motorJ2;
SimpleTimer   controlTimer;
AS5600        encoder;
SimplePID     pid;

volatile float   g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;
volatile float   g_targetJ1  = 0.0f;

// ─── Encoder task ────────────────────────────────────────────────────
void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─── Control loop ────────────────────────────────────────────────────
void controlLoop(void* arg)
{
    float error  = g_targetJ1 - g_encAngle;
    bool  j1Done = fabsf(error) <= TOLERANCE;
    bool  j2Done = motorJ2.atTarget();

    // only cut power when both are done
    if (j1Done && j2Done)
    {
        motorJ1.enable();  // shared pin — disables both
        return;
    }
    motorJ1.enable();  // shared pin — enables both

    // J1 — closed loop PID
    if (!j1Done)
    {
        float output = pid.calc(error);
        if (output > DEADBAND)
        {
            gpio_set_level((gpio_num_t)J1_DIR_PIN, 0);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 0);
        }
        else if (output < -DEADBAND)
        {
            gpio_set_level((gpio_num_t)J1_DIR_PIN, 1);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 0);
        }
    }

    // J2 — open loop
    if (!j2Done)
        motorJ2.update();
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

// ─── Move J1 (closed loop) ────────────────────────────────────────────
void moveJ1(float deg)
{
    ESP_LOGI(TAG, "J1 → %.2f deg", deg);
    g_targetJ1 = deg;
    pid.reset();

    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_targetJ1 - g_encAngle) > TOLERANCE)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(5000))
        {
            ESP_LOGW(TAG, "J1 timeout at %.2f deg", g_encAngle);
            break;
        }
        ESP_LOGI(TAG, "J1 target: %.2f | encoder: %.2f | error: %.2f | magnet: %s",
                 g_targetJ1, g_encAngle, g_targetJ1 - g_encAngle, magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "J1 done — encoder: %.2f", g_encAngle);
}

// ─── Move J2 (open loop) ─────────────────────────────────────────────
void moveJ2(float deg)
{
    ESP_LOGI(TAG, "J2 → %.2f deg", deg);
    motorJ2.setTarget(deg);

    TickType_t start = xTaskGetTickCount();
    while (!motorJ2.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(5000))
        {
            ESP_LOGW(TAG, "J2 timeout at %.2f deg", motorJ2.getAngle());
            break;
        }
        ESP_LOGI(TAG, "J2: %.2f deg", motorJ2.getAngle());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "J2 done — %.2f deg", motorJ2.getAngle());
}

// ─── Entry point ─────────────────────────────────────────────────────
extern "C" void app_main()
{
    motorJ1.setup(J1_STEP_PIN, J1_DIR_PIN, EN_PIN, 200, 1, 3.0f);
    motorJ2.setup(J2_STEP_PIN, J2_DIR_PIN, EN_PIN, 200, 1, 3.0f);  // update gear ratio when known

    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found — check wiring and magnet");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready — magnet: %s", magnetStatus());
    }

    float gains[3] = { 0.1f, 0.0f, 0.005f };
    pid.setup(gains, DT, -1.0f, 1.0f);

    xTaskCreate(encoderTask, "encoder", 2048, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));

    g_targetJ1 = g_encAngle;  // hold J1 at current position

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Two Stepper Test ===");
    moveJ1(45.0f);
    moveJ1(0.0f);


    ESP_LOGI(TAG, "=== Done ===");

    while (true)
    {
        ESP_LOGI(TAG, "J1 enc: %.2f | J2 steps: %.2f | magnet: %s",
                 g_encAngle, motorJ2.getAngle(), magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}