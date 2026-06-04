#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

// Define servo parameters
#define SERVO_PIN               18   // GPIO pin connected to the servo
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds (0 degrees)
#define SERVO_MAX_PULSEWIDTH_US 2400 // Maximum pulse width in microseconds (180 degrees)
#define SERVO_MAX_DEGREE        180  // Maximum angle of the servo

// Define LEDC parameters
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE 
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT // 14-bit resolution (values from 0 to 16383)
#define LEDC_FREQUENCY          50                // SG90 operates at 50Hz

// Helper function to map an angle to a PWM duty cycle
uint32_t angle_to_duty(int angle) {
    // Clamp the angle to prevent commanding out-of-bound movements
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    if (angle < 0) angle = 0;

    // 1. Calculate the pulse width for the requested angle
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
        (((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE);

    // 2. Convert pulse width to duty cycle
    // Period for 50Hz is 20,000 microseconds. 
    // Max duty cycle for 14-bit is 2^14 = 16384
    uint32_t duty = (pulse_width * 16384) / 20000;
    
    return duty;
}

// extern "C" prevents C++ name mangling so the ESP-IDF C framework can find app_main
extern "C" void app_main(void) {
    
    // 1. Configure the LEDC Timer (Zero-initialized for C++ compatibility)
    ledc_timer_config_t ledc_timer = {}; 
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;  // Set output frequency to 50 Hz
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    
    ledc_timer_config(&ledc_timer);

    // 2. Configure the LEDC Channel (Zero-initialized for C++ compatibility)
    ledc_channel_config_t ledc_channel = {}; 
    ledc_channel.gpio_num       = SERVO_PIN;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = LEDC_CHANNEL;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.duty           = 0; // Initialize duty cycle to 0
    ledc_channel.hpoint         = 0;
    
    ledc_channel_config(&ledc_channel);

    printf("Servo configuration complete. Starting sweep...\n");

    // 3. Main Loop
    // 3. Main Loop
    while (1) {
        printf("Sweeping up to 180...\n");
        // Sweep from 0 to 180 degrees
        for (int angle = 0; angle <= SERVO_MAX_DEGREE; angle++) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle_to_duty(angle));
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(15)); 
        }
        
        printf("Sweeping down to 0...\n");
        // Sweep from 180 down to 0 degrees
        for (int angle = SERVO_MAX_DEGREE; angle >= 0; angle--) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, angle_to_duty(angle));
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}