#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Librerías de Motores y Sensores
#include "SimpleStepper.h"
#include "SimpleBDC.h"
#include "SimplePWM.h"
#include "SimpleTimer.h"
#include "AS5600.h"

static const char* TAG = "QuadMotorControl";

// ==========================================
// PINES DE MOTORES DC Y LIMIT SWITCH
// ==========================================
#define LIMIT_SWITCH_PIN 4 
#define M2_DEG_PER_CM 76.92f

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

// ==========================================
// PINES DE MOTORES STEPPER Y AS5600
// ==========================================
#define STEP_1_PIN  5 
#define DIR_1_PIN   18 
#define STEP_2_PIN  19
#define DIR_2_PIN   23
#define EN_PIN      33 
#define SDA_PIN     21
#define SCL_PIN     22

// ==========================================
// CONFIGURACIÓN GLOBAL DE CONTROL
// ==========================================
#define CONTROL_HZ  100
#define CONTROL_US  (1000000 / CONTROL_HZ)
#define CONTROL_DT  (1.0f / CONTROL_HZ)

// Instancias globales
SimpleBDC     motorDC1;
SimpleBDC     motorDC2;
SimpleStepper stepper1;
SimpleStepper stepper2;
SimpleTimer   controlTimer;
AS5600        encoder;

volatile float g_encAngle  = 0.0f;
volatile uint8_t g_encStatus = 0;

// ==========================================
// RUTINAS DE TAREAS Y TIMERS
// ==========================================

// Bucle unificado a 100Hz para los 4 motores
void controlLoop(void* arg)
{
    // Actualizar Steppers
    stepper1.update();  
    stepper2.update();

    // Actualizar DC Motor 1
    motorDC1.update();

    // Lógica del Limit Switch para DC Motor 2
    if (gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN) == 1) 
    {
        if (motorDC2.getTarget() < motorDC2.getAngle()) 
        {
            motorDC2.stop();
        }
    }
    // Actualizar DC Motor 2
    motorDC2.update();
}

void encoderTask(void* arg)
{
    while (true)
    {
        g_encAngle  = encoder.getAngle();
        g_encStatus = encoder.getStatus();
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz
    }
}

const char* magnetStatus()
{
    if (g_encStatus & AS5600_MAGNET_OK)     return "OK";
    if (g_encStatus & AS5600_MAGNET_WEAK)   return "WEAK (too far)";
    if (g_encStatus & AS5600_MAGNET_STRONG) return "STRONG (too close)";
    return "NOT DETECTED";
}

// Función central para enviar metas a todos los motores y esperar a los steppers
void moveAllSimultaneously(float s1_deg, float s2_deg, float dc1_deg, float dc2_cm)
{
    float dc2_deg = dc2_cm * M2_DEG_PER_CM;
    
    ESP_LOGI(TAG, "→ Metas | S1: %.2f° | S2: %.2f° | DC1: %.2f° | DC2: %.2fcm (%.2f°)", 
             s1_deg, s2_deg, dc1_deg, dc2_cm, dc2_deg);
    
    // Activar motores DC de fondo (estos llegarán a su meta asíncronamente con su PID)
    motorDC1.setTarget(dc1_deg);
    motorDC2.setTarget(dc2_deg);

    // Activar Steppers
    stepper1.setTarget(s1_deg);
    stepper2.setTarget(s2_deg);
    
    // Bloquear parser hasta que LOS DOS STEPPERS lleguen (Comportamiento original preservado)
    while (!stepper1.atTarget() || !stepper2.atTarget())
    {
        ESP_LOGI(TAG, "Mov... S1: %.2f | S2: %.2f | DC1: %.2f | DC2(cm): %.2f | AS5600: %.2f", 
                 stepper1.getAngle(), stepper2.getAngle(), 
                 motorDC1.getAngle(), motorDC2.getAngle() / M2_DEG_PER_CM, g_encAngle);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    ESP_LOGI(TAG, "¡Steppers han llegado a su destino!");
}

// ==========================================
// FUNCIÓN PRINCIPAL
// ==========================================
extern "C" void app_main()
{
    // --- 1. SETUP DE MOTORES DC ---
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LIMIT_SWITCH_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;      
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;   
    gpio_config(&io_conf);

    TimerConfig bdcTimer;
    bdcTimer.frequency = 20000;

    float gains[3]      = { 1.0f, 0.1f, 0.05f };
    uint8_t encPins1[2] = { BDC1_ENC_A, BDC1_ENC_B };
    uint8_t encPins2[2] = { BDC2_ENC_A, BDC2_ENC_B };
    
    float degPerEdge = 360.0f / (1026.0f  * 3.0f);    

    motorDC1.setup(BDC1_PWM_PIN, BDC1_IN1_PIN, BDC1_IN2_PIN, encPins1, gains,
                   CONTROL_DT, degPerEdge, &bdcTimer, 0);
    motorDC2.setup(BDC2_PWM_PIN, BDC2_IN1_PIN, BDC2_IN2_PIN, encPins2, gains,
                   CONTROL_DT, degPerEdge, &bdcTimer, 1);

    // --- 2. SETUP DE MOTORES STEPPER ---
    gpio_reset_pin((gpio_num_t)EN_PIN);
    gpio_set_direction((gpio_num_t)EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)EN_PIN, 0); 

    // ¡OJO! Cambié los pines dummy de 27/26 a 16/17 para que no colisionen con el DC.
    stepper1.setup(STEP_1_PIN, DIR_1_PIN, 16, 200, 1, 3.0f);
    stepper2.setup(STEP_2_PIN, DIR_2_PIN, 17, 200, 1, 3.0f);

    // --- 3. SETUP DEL ENCODER ---
    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    if (!encoder.isConnected())
        ESP_LOGW(TAG, "AS5600 no encontrado");
    else {
        encoder.setZero();
        ESP_LOGI(TAG, "AS5600 listo — Magnet: %s", magnetStatus());
    }

    // --- 4. INICIAR TIMERS Y TAREAS ---
    controlTimer.setup(controlLoop, "control");
    controlTimer.startPeriodic(CONTROL_US);

    xTaskCreate(encoderTask, "encoder", 2048, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- 5. BUCLE PRINCIPAL (ENTRADA SERIAL Y STATUS) ---
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    ESP_LOGI(TAG, "=== Sistema de 4 Motores Listo ===");
    ESP_LOGI(TAG, "Escribe en formato: 'x,y,z,w' (S1, S2, DC1, DC2_cm)");
    ESP_LOGI(TAG, "Ejemplo: 90, 45, 180, -5.5");

    char rx_buf[64];
    int rx_idx = 0;
    uint32_t last_print = 0;

    while (true)
    {
        // Revisar Entrada Serial
        int c = fgetc(stdin);
        if (c != EOF && c != 255) {
            if (c == '\n' || c == '\r') {
                if (rx_idx > 0) {
                    rx_buf[rx_idx] = '\0';            
                    
                    float t_x = 0.0f, t_y = 0.0f, t_z = 0.0f, t_w = 0.0f;
                    
                    // Parsear formato 'x,y,z,w'
                    if (sscanf(rx_buf, "%f,%f,%f,%f", &t_x, &t_y, &t_z, &t_w) == 4) {
                        moveAllSimultaneously(t_x, t_y, t_z, t_w);
                    } else {
                        ESP_LOGW(TAG, "Formato Invalido. Usa: 'x,y,z,w'");
                    }
                    
                    rx_idx = 0;                       
                }
            } else if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;                 
            }
        }

        // Impresión periódica (1 Hz)
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        if (current_time - last_print > 1000) {
            int sw_state = gpio_get_level((gpio_num_t)LIMIT_SWITCH_PIN);
            ESP_LOGI(TAG, "[Status] S1: %.2f | S2: %.2f | DC1: %.2f | DC2: %.2fcm | AS5600: %.2f | SW: %d",
                     stepper1.getAngle(), stepper2.getAngle(), 
                     motorDC1.getAngle(), motorDC2.getAngle() / M2_DEG_PER_CM, 
                     g_encAngle, sw_state);
            last_print = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}