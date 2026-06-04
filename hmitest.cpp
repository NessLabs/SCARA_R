#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "SimpleStepper.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "AS5600.h"
#include "SimplePID.h"
#include "Kinematics.h"
#include "RobotState.h"
#include "Network.h"

static const char* TAG = "Robot";

// ─── Pins ─────────────────────────────────────────────────────────────
#define LIMIT_SWITCH_PIN  4
#define M2_DEG_PER_CM     76.92f

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

#define TOLERANCE    2.0f
#define DEADBAND     0.05f
#define JOG_STEP     5.0f   // degrees or mm per jog tick

// ─── Objects ─────────────────────────────────────────────────────────
SimpleBDC     motorDC1, motorDC2;
SimpleStepper stepper1, stepper2;
SimpleTimer   controlTimer;
AS5600        encoder;
SimplePID     pidS1;

// ─── Shared state ─────────────────────────────────────────────────────
volatile float   g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;
volatile float   g_targetS1  = 0.0f;
volatile bool    g_estop     = false;
volatile bool    g_homed     = false;

QueueHandle_t cmdQueue = nullptr;

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
    if (g_estop) return;

    // S1 — closed loop PID
    float errS1  = g_targetS1 - g_encAngle;
    bool  s1Done = fabsf(errS1) <= TOLERANCE;
    bool  s2Done = stepper2.atTarget();

    if (s1Done && s2Done)
        stepper1.disable();
    else
        stepper1.enable();

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

    if (!s2Done) stepper2.update();

    // limit switch for DC2
    if (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 1)
        if (motorDC2.getTarget() < motorDC2.getAngle())
            motorDC2.stop();

    motorDC1.update();
    motorDC2.update();
}

// ─── State reporting task — sends to laptop at 20Hz ──────────────────
void stateTask(void* arg)
{
    while (true)
    {
        updateRobotState(
            g_encAngle,
            stepper2.getAngle(),
            motorDC1.getAngle(),
            motorDC2.getAngle() / M2_DEG_PER_CM,
            0.0f,
            0.0f,
            motorDC1.getSpeed(),
            motorDC2.getSpeed() / M2_DEG_PER_CM
        );

        sendState(
            robotState.joints.theta1,
            robotState.joints.theta2,
            robotState.joints.theta3,
            robotState.joints.d4,
            robotState.velocities.theta1,
            robotState.velocities.theta2,
            robotState.velocities.theta3,
            robotState.velocities.d4
        );

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─── E-Stop ───────────────────────────────────────────────────────────
void triggerEstop()
{
    g_estop = true;
    stepper1.disable();  // cuts both steppers
    motorDC1.stop();
    motorDC2.stop();
    ESP_LOGW(TAG, "E-STOP triggered");
}

void resetEstop()
{
    g_estop = false;
    ESP_LOGI(TAG, "E-STOP reset");
}

// ─── Homing ───────────────────────────────────────────────────────────
void doHome()
{
    ESP_LOGI(TAG, "Homing...");

    // DC2 — move toward limit switch until it triggers
    motorDC2.setTarget(-9999.0f);
    while (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 0)
        vTaskDelay(pdMS_TO_TICKS(20));
    motorDC2.stop();
    motorDC2.reset();
    ESP_LOGI(TAG, "DC2 homed");

    // S1 — move to zero via encoder
    g_targetS1 = 0.0f;
    pidS1.reset();
    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_encAngle) > TOLERANCE)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    encoder.setZero();
    ESP_LOGI(TAG, "S1 homed");

    // S2, DC1 — move to zero
    stepper2.setTarget(0.0f);
    motorDC1.setTarget(0.0f);
    while (!stepper2.atTarget())
        vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "All homed");

    g_homed = true;
}

// ─── Move helpers ─────────────────────────────────────────────────────
void setAllTargets(float t1, float t2, float t3, float d4)
{
    g_targetS1 = t1;
    pidS1.reset();
    stepper2.setTarget(t2);
    motorDC1.setTarget(t3);
    motorDC2.setTarget(d4 * M2_DEG_PER_CM);
}

void waitForArrival()
{
    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_targetS1 - g_encAngle) > TOLERANCE ||
           !stepper2.atTarget() ||
           !motorDC1.atTarget() ||
           !motorDC2.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(10000))
        {
            ESP_LOGW(TAG, "Move timeout");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─── Command processor task ───────────────────────────────────────────
void commandTask(void* arg)
{
    RobotCommand cmd;
    while (true)
    {
        if (xQueueReceive(cmdQueue, &cmd, pdMS_TO_TICKS(10)))
        {
            ESP_LOGI(TAG, "cmd: %s", cmd.cmd);

            if (strcmp(cmd.cmd, "estop") == 0)
            {
                triggerEstop();
            }
            else if (strcmp(cmd.cmd, "estop_reset") == 0)
            {
                resetEstop();
            }
            else if (strcmp(cmd.cmd, "home") == 0)
            {
                doHome();
            }
            else if (g_estop)
            {
                ESP_LOGW(TAG, "E-STOP active — ignoring cmd");
            }
            else if (strcmp(cmd.cmd, "move_joint") == 0)
            {
                setAllTargets(cmd.j1, cmd.j2, cmd.j3, cmd.j4);
                waitForArrival();
            }
            else if (strcmp(cmd.cmd, "move_tcp") == 0)
            {
                // run IK
                OpState target = { cmd.x, cmd.y, cmd.z, cmd.alpha };
                JointState js;
                if (scaraIK(target, js))
                {
                    setAllTargets(js.theta1, js.theta2, js.theta3, js.d4);
                    waitForArrival();
                }
                else
                    ESP_LOGE(TAG, "IK failed — target out of reach");
            }
            else if (strcmp(cmd.cmd, "jog") == 0)
            {
                if (!cmd.active) continue;
                float step = cmd.direction * cmd.step;

                if (strcmp(cmd.axis, "j1") == 0)
                {
                    g_targetS1 += step;
                    pidS1.reset();
                }
                else if (strcmp(cmd.axis, "j2") == 0)
                    stepper2.setTarget(stepper2.getAngle() + step);
                else if (strcmp(cmd.axis, "j3") == 0)
                    motorDC1.setTarget(motorDC1.getAngle() + step);
                else if (strcmp(cmd.axis, "j4") == 0)
                    motorDC2.setTarget(motorDC2.getAngle() + step * M2_DEG_PER_CM);
                else
                {
                    // TCP jog — compute IK from current + delta
                    OpState cur = robotState.tcp;
                    if      (strcmp(cmd.axis, "x")     == 0) cur.x     += step;
                    else if (strcmp(cmd.axis, "y")     == 0) cur.y     += step;
                    else if (strcmp(cmd.axis, "z")     == 0) cur.z     += step;
                    else if (strcmp(cmd.axis, "alpha") == 0) cur.phi   += step;

                    JointState js;
                    if (scaraIK(cur, js))
                    {
                        g_targetS1 = js.theta1;
                        pidS1.reset();
                        stepper2.setTarget(js.theta2);
                        motorDC1.setTarget(js.theta3);
                        motorDC2.setTarget(js.d4 * M2_DEG_PER_CM);
                    }
                }
            }
        }
    }
}

// ─── Entry point ─────────────────────────────────────────────────────
extern "C" void app_main()
{
    // limit switch
    gpio_config_t sw = {};
    sw.intr_type    = GPIO_INTR_DISABLE;
    sw.mode         = GPIO_MODE_INPUT;
    sw.pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN);
    sw.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&sw);

    // DC motors
    TimerConfig bdcTimer;
    bdcTimer.frequency  = 20000;
    float gains[3]      = { 1.0f, 0.1f, 0.05f };
    uint8_t encPins1[2] = { BDC1_ENC_A, BDC1_ENC_B };
    uint8_t encPins2[2] = { BDC2_ENC_A, BDC2_ENC_B };
    float degPerEdge    = 360.0f / (1026.0f * 3.0f);

    motorDC1.setup(BDC1_PWM_PIN, BDC1_IN1_PIN, BDC1_IN2_PIN, encPins1,
                   gains, CONTROL_DT, degPerEdge, &bdcTimer, 0);
    motorDC2.setup(BDC2_PWM_PIN, BDC2_IN1_PIN, BDC2_IN2_PIN, encPins2,
                   gains, CONTROL_DT, degPerEdge, &bdcTimer, 1);

    // stepper GPIO
    gpio_config_t sc = {};
    sc.mode         = GPIO_MODE_OUTPUT;
    sc.intr_type    = GPIO_INTR_DISABLE;
    sc.pin_bit_mask = (1ULL << STEP_1_PIN) | (1ULL << DIR_1_PIN)
                    | (1ULL << STEP_2_PIN) | (1ULL << DIR_2_PIN)
                    | (1ULL << EN_PIN);
    gpio_config(&sc);

    stepper1.setup(STEP_1_PIN, DIR_1_PIN, EN_PIN, 200, 1, 3.0f);
    stepper2.setup(STEP_2_PIN, DIR_2_PIN, EN_PIN, 200, 1, 1.0f);

    // PID for S1
    float pidGains[3] = { 0.05f, 0.0f, 0.005f };
    pidS1.setup(pidGains, CONTROL_DT, -1.0f, 1.0f);

    // AS5600
    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready");
    }

    // network — init WiFi and wait for connection before starting tasks
    cmdQueue = networkInit();

    // wait for WiFi IP — check every 500ms up to 30 seconds
    ESP_LOGI(TAG, "Waiting for WiFi...");
    bool wifiReady = false;
    for (int i = 0; i < 60; i++)
    {
        esp_netif_ip_info_t ip;
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0)
        {
            ESP_LOGI(TAG, "WiFi connected — IP: " IPSTR, IP2STR(&ip.ip));
            wifiReady = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!wifiReady)
        ESP_LOGW(TAG, "WiFi not connected — running without network");

    // tasks
    xTaskCreate(encoderTask,  "encoder",  2048, NULL, 5, NULL);
    xTaskCreate(stateTask,    "state",    4096, NULL, 3, NULL);
    xTaskCreate(commandTask,  "command",  4096, NULL, 4, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));

    g_targetS1 = g_encAngle;

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);

    ESP_LOGI(TAG, "=== Robot ready — waiting for HMI commands ===");

    // main task just monitors
    while (true)
    {
        ESP_LOGI(TAG, "S1: %.2f | S2: %.2f | DC1: %.2f | DC2: %.2fmm | estop: %d",
                 g_encAngle,
                 stepper2.getAngle(),
                 motorDC1.getAngle(),
                 motorDC2.getAngle() / M2_DEG_PER_CM,
                 (int)g_estop);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}