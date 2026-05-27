#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "SimpleStepper.h"
#include "SimpleTimer.h"
#include "AS5600.h"

static const char* TAG = "SCARA_STABLE";

#define STEP_PIN    5 //25  
#define DIR_PIN     18 //26
#define EN_PIN      33 //27
#define SDA_PIN     21
#define SCL_PIN     22

#define CONTROL_HZ  100
#define CONTROL_US  (1000000 / CONTROL_HZ)

// ─── TUNED FOR STABILITY ───
// Lowered Kp (0.4) and Kd (0.01). Setting Ki to 0 for now to stop the "crawling"
float pidGains[3] = {0.4f, 0.0f, 0.01f}; 
float myTolerance = 2.0f; // <--- Your 2 degree tolerance

SimpleStepper stepper;
SimpleTimer   controlTimer;
AS5600        encoder;

volatile float g_encAngle = 0.0f;

// Unwrapping logic to handle 0-360 jumps
void encoderTask(void* arg) {
    float lastRaw = encoder.getAngle();
    float cumulative = 0;
    while (1) {
        float current = encoder.getAngle();
        float delta = current - lastRaw;
        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;
        cumulative += delta;
        g_encAngle = cumulative;
        lastRaw = current;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void controlLoop(void* arg) {
    stepper.updatePID(g_encAngle);
}

void moveTo(float deg) {
    ESP_LOGI(TAG, "Moving to %.2f...", deg);
    stepper.setTarget(deg);
    while (!stepper.atTargetPID(g_encAngle)) {
        ESP_LOGI(TAG, "Pos: %.2f | Err: %.2f", g_encAngle, deg - g_encAngle);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "Stayed within tolerance!");
}

extern "C" void app_main() {
    // Setup with the tuned gains
    stepper.setup(STEP_PIN, DIR_PIN, EN_PIN, 200, 1, 3.0f, pidGains, 1.0f/CONTROL_HZ, false);
    stepper.setTolerance(myTolerance);

    encoder.setup(SDA_PIN, SCL_PIN, 3.0f);
    encoder.setZero();

    xTaskCreate(encoderTask, "enc", 2048, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));

    controlTimer.setup(controlLoop, "pid");
    controlTimer.startPeriodic(CONTROL_US);

    moveTo(45.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    moveTo(0.0f);

    while(1) vTaskDelay(1000);
}