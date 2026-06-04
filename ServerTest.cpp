#include "Network.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ServerTest";

extern "C" void app_main()
{
    // init WiFi + UDP, returns command queue
    QueueHandle_t cmdQueue = networkInit();

    vTaskDelay(pdMS_TO_TICKS(5000));  // wait for WiFi to connect
    ESP_LOGI(TAG, "Network ready");

    RobotCommand cmd;

    while (true)
    {
        // check for incoming commands
        if (xQueueReceive(cmdQueue, &cmd, 0))
        {
            ESP_LOGI(TAG, "cmd: %s | j1=%.2f j2=%.2f j3=%.2f j4=%.2f",
                     cmd.cmd, cmd.j1, cmd.j2, cmd.j3, cmd.j4);
        }

        // send dummy state back to laptop every 100ms
        sendState(10.0f, 20.0f, 150.0f, 0.0f);
        ESP_LOGI(TAG, "Dummy");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}