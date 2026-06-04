#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

#include "SimpleStepper.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "AS5600.h"
#include "SimplePID.h"

static const char* TAG = "QuadMotorControl";

// ─── DC motor pins ────────────────────────────────────────────────────
#define LIMIT_SWITCH_PIN 4
#define M2_DEG_PER_CM    76.92f

#define BDC1_PWM_PIN  13
#define BDC1_IN1_PIN  14
#define BDC1_IN2_PIN  27
#define BDC1_ENC_A    36
#define BDC1_ENC_B    39

#define BDC2_PWM_PIN  32
#define BDC2_IN1_PIN  25
#define BDC2_IN2_PIN  26
#define BDC2_ENC_A    34
#define BDC2_ENC_B    35

// ─── Stepper pins ─────────────────────────────────────────────────────
#define STEP_1_PIN   5
#define DIR_1_PIN    18
#define STEP_2_PIN   19
#define DIR_2_PIN    23
#define EN_PIN       33
#define SDA_PIN      21
#define SCL_PIN      22

// ─── Control timing ───────────────────────────────────────────────────
#define CONTROL_HZ   500
#define CONTROL_US   (1000000 / CONTROL_HZ)
#define CONTROL_DT   (1.0f / CONTROL_HZ)

// ─── PID stepper config ───────────────────────────────────────────────
#define TOLERANCE    2.0f
#define DEADBAND     0.5f

// ─── Objects ─────────────────────────────────────────────────────────
SimpleBDC     motorDC1, motorDC2;
SimpleStepper stepper1, stepper2;   // used only for enable/disable
SimpleTimer   controlTimer;
AS5600        encoder;
SimplePID     pidS1;                // PID for stepper 1 (AS5600 feedback)
SimplePID     pidS2;                // PID for stepper 2 (open loop via step count)

// ─── Shared state ─────────────────────────────────────────────────────
volatile float   g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;
volatile float   g_targetS1  = 0.0f;  // J1 target (closed loop)
volatile float   g_targetS2  = 0.0f;  // J2 target (open loop)

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
    // ── Stepper 1 — closed loop PID using AS5600 ──
    float errS1 = g_targetS1 - g_encAngle;
    bool  s1Done = fabsf(errS1) <= TOLERANCE;

    // ── Stepper 2 — open loop ──
    bool  s2Done = stepper2.atTarget();

    // shared EN pin — only cut power when both steppers are done
    if (s1Done && s2Done)
        stepper1.disable();  // disables both
    else
        stepper1.enable();   // enables both

    // S1 PID step
    if (!s1Done)
    {
        float output = pidS1.calc(errS1);
        if (output > DEADBAND)
        {
            gpio_set_level((gpio_num_t)DIR_1_PIN, 1);
            gpio_set_level((gpio_num_t)STEP_1_PIN, 1);
            ets_delay_us(5);
            gpio_set_level((gpio_num_t)STEP_1_PIN, 0);
        }
        else if (output < -DEADBAND)
        {
            gpio_set_level((gpio_num_t)DIR_1_PIN, 0);
            gpio_set_level((gpio_num_t)STEP_1_PIN, 1);
            ets_delay_us(5);
            gpio_set_level((gpio_num_t)STEP_1_PIN, 0);
        }
    }

    // S2 open loop
    if (!s2Done)
        stepper2.update();

    // DC motor 1
    motorDC1.update();

    // DC motor 2 with limit switch
    if (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 1)
    {
        if (motorDC2.getTarget() < motorDC2.getAngle())
            motorDC2.stop();
    }
    motorDC2.update();
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

// ─── Move all motors simultaneously ──────────────────────────────────
void moveAllSimultaneously(float s1_deg, float s2_deg, float dc1_deg, float dc2_cm)
{
    float dc2_deg = dc2_cm * M2_DEG_PER_CM;

    ESP_LOGI(TAG, "→ S1: %.2f° | S2: %.2f° | DC1: %.2f° | DC2: %.2fcm",
             s1_deg, s2_deg, dc1_deg, dc2_cm);

    // DC motors — async, PID handles arrival
    motorDC1.setTarget(dc1_deg);
    motorDC2.setTarget(dc2_deg);

    // S1 — closed loop
    g_targetS1 = s1_deg;
    pidS1.reset();

    // S2 — open loop
    g_targetS2 = s2_deg;
    stepper2.setTarget(s2_deg);

    // wait for both steppers
    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_targetS1 - g_encAngle) > TOLERANCE || !stepper2.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000))
        {
            ESP_LOGW(TAG, "timeout — S1: %.2f | S2: %.2f",
                     g_encAngle, stepper2.getAngle());
            break;
        }
        ESP_LOGI(TAG, "S1: %.2f/%.2f | S2: %.2f/%.2f | DC1: %.2f | DC2: %.2fcm | mag: %s",
                 g_encAngle, g_targetS1,
                 stepper2.getAngle(), g_targetS2,
                 motorDC1.getAngle(),
                 motorDC2.getAngle() / M2_DEG_PER_CM,
                 magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Steppers at target");
}

// ─── Entry point ─────────────────────────────────────────────────────
extern "C" void app_main()
{
    // ── Limit switch ──
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN);
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    // ── DC motors ──
    TimerConfig bdcTimer;
    bdcTimer.frequency = 20000;

    float gains[3]      = { 1.0f, 0.1f, 0.05f };
    uint8_t encPins1[2] = { BDC1_ENC_A, BDC1_ENC_B };
    uint8_t encPins2[2] = { BDC2_ENC_A, BDC2_ENC_B };
    float degPerEdge    = 360.0f / (1026.0f * 3.0f);

    motorDC1.setup(BDC1_PWM_PIN, BDC1_IN1_PIN, BDC1_IN2_PIN, encPins1, gains,
                   CONTROL_DT, degPerEdge, &bdcTimer, 0);
    motorDC2.setup(BDC2_PWM_PIN, BDC2_IN1_PIN, BDC2_IN2_PIN, encPins2, gains,
                   CONTROL_DT, degPerEdge, &bdcTimer, 1);

    // ── Stepper GPIO (EN shared, steppers handle STEP/DIR directly in controlLoop) ──
    gpio_config_t sconf = {};
    sconf.mode         = GPIO_MODE_OUTPUT;
    sconf.intr_type    = GPIO_INTR_DISABLE;
    sconf.pin_bit_mask = (1ULL << STEP_1_PIN) | (1ULL << DIR_1_PIN)
                       | (1ULL << STEP_2_PIN) | (1ULL << DIR_2_PIN)
                       | (1ULL << EN_PIN);
    gpio_config(&sconf);

    // stepper1 used only for enable/disable (shared EN)
    stepper1.setup(STEP_1_PIN, DIR_1_PIN, EN_PIN, 200, 1, 3.0f);
    // stepper2 open loop — uses its own update()
    stepper2.setup(STEP_2_PIN, DIR_2_PIN, EN_PIN, 200, 1, 1.0f);

    // ── PID for S1 ──
    float pidGains[3] = { 0.05f, 0.0f, 0.005f };
    pidS1.setup(pidGains, CONTROL_DT, -1.0f, 1.0f);

    // ── AS5600 ──
    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready — magnet: %s", magnetStatus());
    }

    // ── Start tasks ──
    xTaskCreate(encoderTask, "encoder", 2048, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));

    g_targetS1 = g_encAngle;  // hold S1 at current position

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);
    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Serial input loop ──
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    ESP_LOGI(TAG, "=== 4-Motor System Ready ===");
    ESP_LOGI(TAG, "Format: 'S1_deg, S2_deg, DC1_deg, DC2_cm'");
    ESP_LOGI(TAG, "Example: 90, 45, 180, -5.5");    
 
    char     rx_buf[64];
    int      rx_idx   = 0;
    uint32_t lastPrint = 0;

    while (true)
    {
        int c = fgetc(stdin);
        if (c != EOF && c != 255)
        {
            if (c == '\n' || c == '\r')
            {
                if (rx_idx > 0)
                {
                    rx_buf[rx_idx] = '\0';
                    float t_s1=0, t_s2=0, t_dc1=0, t_dc2=0;
                    if (sscanf(rx_buf, "%f,%f,%f,%f", &t_s1, &t_s2, &t_dc1, &t_dc2) == 4)
                        moveAllSimultaneously(t_s1, t_s2, t_dc1, t_dc2);
                    else
                        ESP_LOGW(TAG, "Invalid format — use: S1,S2,DC1,DC2_cm");
                    rx_idx = 0;
                }
            }
            else if (rx_idx < (int)sizeof(rx_buf) - 1)
                rx_buf[rx_idx++] = c;
        }

        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        if (now - lastPrint > 1000)
        {
            ESP_LOGI(TAG, "[Status] S1: %.2f | S2: %.2f | DC1: %.2f | DC2: %.2fcm | AS5600: %.2f | SW: %d",
                     g_encAngle, stepper2.getAngle(),
                     motorDC1.getAngle(), motorDC2.getAngle() / M2_DEG_PER_CM,
                     g_encAngle, gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN));
            lastPrint = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}