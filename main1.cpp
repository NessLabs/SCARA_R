#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
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

#define BDC_Z_PWM_PIN     13
#define BDC_Z_IN1_PIN     14
#define BDC_Z_IN2_PIN     27
#define BDC_Z_ENC_A       36
#define BDC_Z_ENC_B       39

#define BDC_ROT_PWM_PIN   32
#define BDC_ROT_IN1_PIN   25
#define BDC_ROT_IN2_PIN   26
#define BDC_ROT_ENC_A     34
#define BDC_ROT_ENC_B     35

#define J1_STEP_PIN       5
#define J1_DIR_PIN        18
#define J2_STEP_PIN       19
#define J2_DIR_PIN        23
#define EN_PIN            33   // shared enable both steppers

#define SDA_PIN           21
#define SCL_PIN           22

// ─── Physical constants ───────────────────────────────────────────────
#define Z_PICK_MM         50.0f
#define Z_PLACE_MM        50.0f
#define GRIPPER_OPEN      0.0f
#define GRIPPER_CLOSE     45.0f
#define DEG_PER_MM_Z      (1026.0f * 3.0f / 360.0f)

struct DropPos { float x, y, phi; };
const DropPos DROP_TRIANGLE = { 150.0f,  80.0f, 0.0f };
const DropPos DROP_SQUARE   = { 150.0f, -80.0f, 0.0f };
const DropPos DROP_HEXAGON  = {-150.0f,  80.0f, 0.0f };
const DropPos DROP_CROSS    = {-150.0f, -80.0f, 0.0f };

// ─── Control timing ───────────────────────────────────────────────────
#define CONTROL_HZ   500
#define CONTROL_US   (1000000 / CONTROL_HZ)
#define CONTROL_DT   (1.0f / CONTROL_HZ)
#define TOLERANCE    2.0f
#define DEADBAND     0.05f

// ─── State machine ────────────────────────────────────────────────────
enum RobotMode  { MODE_MANUAL, MODE_AUTO };
enum RobotPhase {
    PHASE_IDLE,
    PHASE_HOMING,
    PHASE_WAIT_CMD,
    PHASE_ALIGN_PICK,
    PHASE_Z_DOWN_PICK,
    PHASE_CLOSE_GRIPPER,
    PHASE_Z_UP,
    PHASE_ALIGN_PLACE,
    PHASE_Z_DOWN_PLACE,
    PHASE_OPEN_GRIPPER,
    PHASE_Z_UP_FINAL,
    PHASE_DONE
};

static const char* phaseNames[] = {
    "IDLE","HOMING","WAIT_CMD","ALIGN_PICK","Z_DOWN_PICK",
    "CLOSE_GRIPPER","Z_UP","ALIGN_PLACE","Z_DOWN_PLACE",
    "OPEN_GRIPPER","Z_UP_FINAL","DONE"
};

// ─── Objects ─────────────────────────────────────────────────────────
SimpleBDC     motorZ, motorRot;
SimpleStepper motorJ1, motorJ2;
SimpleTimer   controlTimer;
AS5600        encoder;
SimplePID     pid;

// ─── Shared state ─────────────────────────────────────────────────────
volatile float   g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;
volatile float   g_targetJ1  = 0.0f;
volatile bool    g_estop     = false;

RobotMode  mode  = MODE_AUTO;
RobotPhase phase = PHASE_IDLE;

struct PickCommand {
    float x, y, phi;
    char  pieceType[16];
    bool  ready;
} pendingCmd = { 0, 0, 0, "", false };

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

// ─── Control loop — exact pattern from working stepper code ──────────
void controlLoop(void* arg)
{
    if (g_estop) return;

    float error  = g_targetJ1 - g_encAngle;
    bool  j1Done = fabsf(error) <= TOLERANCE;
    bool  j2Done = motorJ2.atTarget();

    // only cut power when both done
    if (j1Done && j2Done)
    {
        motorJ1.disable();  // shared EN — disables both
        return;
    }
    motorJ1.enable();  // shared EN — enables both

    // J1 — closed loop PID
    if (!j1Done)
    {
        float output = pid.calc(error);
        if (output > DEADBAND)
        {
            gpio_set_level((gpio_num_t)J1_DIR_PIN, 1);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 0);
        }
        else if (output < -DEADBAND)
        {
            gpio_set_level((gpio_num_t)J1_DIR_PIN, 0);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level((gpio_num_t)J1_STEP_PIN, 0);
        }
    }

    // J2 — open loop
    if (!j2Done)
        motorJ2.update();

    // Z limit switch
    if (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 1)
        if (motorZ.getTarget() < motorZ.getAngle())
            motorZ.stop();

    motorZ.update();
    motorRot.update();
}

// ─── State reporting ─────────────────────────────────────────────────
void stateTask(void* arg)
{
    while (true)
    {
        updateRobotState(
            g_encAngle,
            motorJ2.getAngle(),
            motorRot.getAngle(),
            motorZ.getAngle() / DEG_PER_MM_Z,
            0.0f, 0.0f,
            motorRot.getSpeed(),
            motorZ.getSpeed() / DEG_PER_MM_Z
        );
        sendState(
            robotState.joints.theta1, robotState.joints.theta2,
            robotState.joints.theta3, robotState.joints.d4,
            robotState.velocities.theta1, robotState.velocities.theta2,
            robotState.velocities.theta3, robotState.velocities.d4
        );
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

// ─── Move helpers ─────────────────────────────────────────────────────
void moveJ1(float deg)
{
    g_targetJ1 = deg;
    pid.reset();
    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_targetJ1 - g_encAngle) > TOLERANCE)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void moveJ2(float deg)
{
    motorJ2.setTarget(deg);
    TickType_t start = xTaskGetTickCount();
    while (!motorJ2.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void moveZ(float mm)
{
    motorZ.setTarget(mm * DEG_PER_MM_Z);
    TickType_t start = xTaskGetTickCount();
    while (!motorZ.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void moveRot(float deg)
{
    motorRot.setTarget(deg);
    TickType_t start = xTaskGetTickCount();
    while (!motorRot.atTarget())
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(5000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void homeZ()
{
    motorZ.setTarget(-99999.0f);
    while (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 0)
        vTaskDelay(pdMS_TO_TICKS(20));
    motorZ.stop();
    motorZ.reset();
}

// ─── E-stop ───────────────────────────────────────────────────────────
void triggerEstop()
{
    g_estop = true;
    motorJ1.disable();
    motorZ.stop();
    motorRot.stop();
    ESP_LOGW(TAG, "E-STOP");
}

void resetEstop() { g_estop = false; }

// ─── Homing ───────────────────────────────────────────────────────────
void doHome()
{
    ESP_LOGI(TAG, "[HOME] Starting");
    moveRot(GRIPPER_OPEN);
    homeZ();
    ESP_LOGI(TAG, "[HOME] Z done");

    // J1 via encoder
    g_targetJ1 = 0.0f;
    pid.reset();
    TickType_t start = xTaskGetTickCount();
    while (fabsf(g_encAngle) > TOLERANCE)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(8000)) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    encoder.setZero();
    ESP_LOGI(TAG, "[HOME] J1 done");

    moveJ2(0.0f);
    ESP_LOGI(TAG, "[HOME] Complete");
}

// ─── Drop position lookup ─────────────────────────────────────────────
DropPos getDropPos(const char* type)
{
    if      (strcmp(type, "triangle") == 0) return DROP_TRIANGLE;
    else if (strcmp(type, "square")   == 0) return DROP_SQUARE;
    else if (strcmp(type, "hexagon")  == 0) return DROP_HEXAGON;
    else if (strcmp(type, "cross")    == 0) return DROP_CROSS;
    return DROP_SQUARE;
}

// ─── State machine ────────────────────────────────────────────────────
void stateMachineTask(void* arg)
{
    while (true)
    {
        if (mode != MODE_AUTO || g_estop)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "[SM] %s", phaseNames[phase]);

        switch (phase)
        {
            case PHASE_IDLE:
                phase = PHASE_HOMING;
                break;

            case PHASE_HOMING:
                doHome();
                phase = PHASE_WAIT_CMD;
                break;

            case PHASE_WAIT_CMD:
                ESP_LOGI(TAG, "[SM] Waiting for command...");
                while (!pendingCmd.ready && !g_estop)
                    vTaskDelay(pdMS_TO_TICKS(100));
                if (g_estop) break;
                pendingCmd.ready = false;
                ESP_LOGI(TAG, "[SM] cmd: x=%.1f y=%.1f type=%s",
                         pendingCmd.x, pendingCmd.y, pendingCmd.pieceType);
                phase = PHASE_ALIGN_PICK;
                break;

            case PHASE_ALIGN_PICK:
            {
                OpState target = { pendingCmd.x, pendingCmd.y, 0.0f, pendingCmd.phi };
                JointState js;
                if (!scaraIK(target, js))
                {
                    ESP_LOGE(TAG, "[SM] IK failed");
                    phase = PHASE_WAIT_CMD;
                    break;
                }
                moveRot(GRIPPER_OPEN);
                moveJ1(js.theta1);
                moveJ2(js.theta2);
                phase = PHASE_Z_DOWN_PICK;
                break;
            }

            case PHASE_Z_DOWN_PICK:
                moveZ(Z_PICK_MM);
                phase = PHASE_CLOSE_GRIPPER;
                break;

            case PHASE_CLOSE_GRIPPER:
                moveRot(GRIPPER_CLOSE);
                vTaskDelay(pdMS_TO_TICKS(500));
                phase = PHASE_Z_UP;
                break;

            case PHASE_Z_UP:
                homeZ();
                phase = PHASE_ALIGN_PLACE;
                break;

            case PHASE_ALIGN_PLACE:
            {
                DropPos dp = getDropPos(pendingCmd.pieceType);
                OpState target = { dp.x, dp.y, 0.0f, dp.phi };
                JointState js;
                if (!scaraIK(target, js))
                {
                    ESP_LOGE(TAG, "[SM] IK failed for place");
                    phase = PHASE_WAIT_CMD;
                    break;
                }
                moveJ1(js.theta1);
                moveJ2(js.theta2);
                phase = PHASE_Z_DOWN_PLACE;
                break;
            }

            case PHASE_Z_DOWN_PLACE:
                moveZ(Z_PLACE_MM);
                phase = PHASE_OPEN_GRIPPER;
                break;

            case PHASE_OPEN_GRIPPER:
                moveRot(GRIPPER_OPEN);
                vTaskDelay(pdMS_TO_TICKS(500));
                phase = PHASE_Z_UP_FINAL;
                break;

            case PHASE_Z_UP_FINAL:
                homeZ();
                phase = PHASE_DONE;
                break;

            case PHASE_DONE:
                ESP_LOGI(TAG, "[SM] Cycle complete");
                phase = PHASE_WAIT_CMD;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─── Command task (from HMI) ──────────────────────────────────────────
void commandTask(void* arg)
{
    RobotCommand cmd;
    while (true)
    {
        if (xQueueReceive(cmdQueue, &cmd, pdMS_TO_TICKS(10)))
        {
            ESP_LOGI(TAG, "cmd: %s | j1=%.2f j2=%.2f j3=%.2f j4=%.2f",
                     cmd.cmd, cmd.j1, cmd.j2, cmd.j3, cmd.j4);

            if (strcmp(cmd.cmd, "estop") == 0)
                triggerEstop();
            else if (strcmp(cmd.cmd, "estop_reset") == 0)
                resetEstop();
            else if (strcmp(cmd.cmd, "home") == 0)
                phase = PHASE_HOMING;
            else if (strcmp(cmd.cmd, "pick") == 0)
            {
                pendingCmd.x   = cmd.x;
                pendingCmd.y   = cmd.y;
                pendingCmd.phi = cmd.alpha;
                strncpy(pendingCmd.pieceType, cmd.axis, sizeof(pendingCmd.pieceType));
                pendingCmd.ready = true;
            }
            else if (strcmp(cmd.cmd, "mode") == 0)
            {
                mode = (strcmp(cmd.axis, "auto") == 0) ? MODE_AUTO : MODE_MANUAL;
                ESP_LOGI(TAG, "Mode: %s", mode == MODE_AUTO ? "AUTO" : "MANUAL");
            }
        }
    }
}

// ─── Serial input task (testing without HMI) ─────────────────────────
// Format: x,y,type   e.g.  100,80,square
void serialTask(void* arg)
{
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = 115200;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    char    buf[64];
    uint8_t idx = 0;
    uint8_t c;

    while (true)
    {
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10)) > 0)
        {
            if (c == '\n' || c == '\r')
            {
                if (idx > 0)
                {
                    buf[idx] = '\0';
                    float x = 0, y = 0;
                    char  type[16] = "square";
                    if (sscanf(buf, "%f,%f,%15s", &x, &y, type) >= 2)
                    {
                        pendingCmd.x   = x;
                        pendingCmd.y   = y;
                        pendingCmd.phi = 0.0f;
                        strncpy(pendingCmd.pieceType, type, sizeof(pendingCmd.pieceType));
                        pendingCmd.ready = true;
                        ESP_LOGI(TAG, "[Serial] x=%.1f y=%.1f type=%s", x, y, type);
                    }
                    else
                        ESP_LOGW(TAG, "Format: x,y,type  e.g. 100,80,square");
                    idx = 0;
                }
            }
            else if (idx < sizeof(buf) - 1)
                buf[idx++] = (char)c;
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
    bdcTimer.frequency    = 20000;
    float gainsZ[3]       = { 1.0f, 0.1f, 0.05f };
    float gainsRot[3]     = { 1.0f, 0.1f, 0.05f };
    uint8_t encZ[2]       = { BDC_Z_ENC_A,   BDC_Z_ENC_B   };
    uint8_t encRot[2]     = { BDC_ROT_ENC_A, BDC_ROT_ENC_B };
    float degPerEdge      = 360.0f / (1026.0f * 3.0f);

    motorZ.setup(BDC_Z_PWM_PIN,   BDC_Z_IN1_PIN,   BDC_Z_IN2_PIN,
                 encZ,   gainsZ,   CONTROL_DT, degPerEdge, &bdcTimer, 0);
    motorRot.setup(BDC_ROT_PWM_PIN, BDC_ROT_IN1_PIN, BDC_ROT_IN2_PIN,
                   encRot, gainsRot, CONTROL_DT, degPerEdge, &bdcTimer, 1);

    // stepper GPIO
    gpio_config_t sc = {};
    sc.mode         = GPIO_MODE_OUTPUT;
    sc.intr_type    = GPIO_INTR_DISABLE;
    sc.pin_bit_mask = (1ULL << J1_STEP_PIN) | (1ULL << J1_DIR_PIN)
                    | (1ULL << J2_STEP_PIN) | (1ULL << J2_DIR_PIN)
                    | (1ULL << EN_PIN);
    gpio_config(&sc);

    motorJ1.setup(J1_STEP_PIN, J1_DIR_PIN, EN_PIN, 200, 1, 3.0f);
    motorJ2.setup(J2_STEP_PIN, J2_DIR_PIN, EN_PIN, 200, 1, 1.0f);

    // PID for J1 — same gains as working code
    float gains[3] = { 0.1f, 0.0f, 0.05f };
    pid.setup(gains, CONTROL_DT, -1.0f, 1.0f);

    // AS5600
    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 not found");
    else
    {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 ready — magnet: %s", magnetStatus());
    }

    // network
    cmdQueue = networkInit();
    ESP_LOGI(TAG, "Waiting for WiFi...");
    bool wifiReady = false;
    for (int i = 0; i < 60; i++)
    {
        esp_netif_ip_info_t ip;
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0)
        {
            ESP_LOGI(TAG, "WiFi — IP: " IPSTR, IP2STR(&ip.ip));
            wifiReady = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!wifiReady)
        ESP_LOGW(TAG, "WiFi not connected");
    else
        udpInit();

    // start tasks
    xTaskCreate(encoderTask,      "encoder", 2048, NULL, 5, NULL);
    xTaskCreate(stateTask,        "state",   4096, NULL, 3, NULL);
    xTaskCreate(commandTask,      "command", 4096, NULL, 4, NULL);
    xTaskCreate(stateMachineTask, "sm",      4096, NULL, 4, NULL);
    xTaskCreate(serialTask,       "serial",  4096, NULL, 3, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));
    g_targetJ1 = g_encAngle;

    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);

    ESP_LOGI(TAG, "=== Robot ready ===");
    ESP_LOGI(TAG, "Serial input: x,y,type  e.g.  100,80,square");
    ESP_LOGI(TAG, "Types: triangle | square | hexagon | cross");

    while (true)
    {
        ESP_LOGI(TAG, "[%s][%s] J1:%.1f J2:%.1f Z:%.1fmm Rot:%.1f | mag:%s",
                 mode == MODE_AUTO ? "AUTO" : "MANUAL",
                 phaseNames[phase],
                 g_encAngle,
                 motorJ2.getAngle(),
                 motorZ.getAngle() / DEG_PER_MM_Z,
                 motorRot.getAngle(),
                 magnetStatus());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}