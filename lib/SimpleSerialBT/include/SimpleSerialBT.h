#ifndef SIMPLE_SERIAL_BT_H
#define SIMPLE_SERIAL_BT_H

#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"

class SerialBT {
public:
    SerialBT();
    bool begin(const char *deviceName);
    int available();
    int read(char *buffer, size_t length);
    size_t write(char *data, size_t length);

private:
    static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    static void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    static uint8_t _rx_buffer[128];
    static int _rx_buffer_head;
    static int _rx_buffer_tail;
    static uint32_t btHandle;
    static bool flag_write;
    static esp_spp_cfg_t bt_spp_cfg;
};

#endif // SIMPLE_SERIAL_BT_H
