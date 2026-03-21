#include "SimpleSerialBT.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SerialBT";

uint8_t SerialBT::_rx_buffer[128];
int SerialBT::_rx_buffer_head = 0;
int SerialBT::_rx_buffer_tail = 0;
uint32_t SerialBT::btHandle;
bool SerialBT::flag_write = false;

esp_spp_cfg_t SerialBT::bt_spp_cfg = {
    .mode = ESP_SPP_MODE_CB,
    .enable_l2cap_ertm = true,
    .tx_buffer_size = 0,
};

SerialBT::SerialBT()
{
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
}

bool SerialBT::begin(const char *deviceName)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    ret = esp_bluedroid_enable();
    ret = esp_bt_gap_register_callback(bt_gap_callback);

    // Register SPP callback
    esp_spp_register_callback(spp_callback);
    ret = esp_spp_enhanced_init(&bt_spp_cfg);
    // Set device name
    esp_bt_gap_set_device_name(deviceName);
    // Set discoverability and connectability
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // Set scan mode to be discoverable and connectable
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "Bluetooth initialized with device name: %s", deviceName);
    esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 0, deviceName);
    return true;
}

int SerialBT::available()
{
    return (_rx_buffer_head >= _rx_buffer_tail) ? (_rx_buffer_head - _rx_buffer_tail) : (sizeof(_rx_buffer) - _rx_buffer_tail + _rx_buffer_head);
}

int SerialBT::read(char *buffer, size_t length)
{
    if (_rx_buffer_head == _rx_buffer_tail)
        return 0; // Buffer empty

    size_t count = 0;
    while (count < length && _rx_buffer_head != _rx_buffer_tail)
    {
        buffer[count++] = _rx_buffer[_rx_buffer_tail];
        _rx_buffer_tail = (_rx_buffer_tail + 1) % sizeof(_rx_buffer);
    }
    return count;
}

size_t SerialBT::write(char *data, size_t length)
{
    if (flag_write)
        esp_spp_write(btHandle, length, (uint8_t *)data); // Assume 0 is the handle for simplicity
    return length;
}

void SerialBT::spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    printf("spp %d\n", event);
    switch (event)
    {
    case ESP_SPP_DATA_IND_EVT:
        for (int i = 0; i < param->data_ind.len; ++i)
        {
            _rx_buffer[_rx_buffer_head] = param->data_ind.data[i];
            _rx_buffer_head = (_rx_buffer_head + 1) % sizeof(_rx_buffer);
            if (_rx_buffer_head == _rx_buffer_tail)
            {
                _rx_buffer_tail = (_rx_buffer_tail + 1) % sizeof(_rx_buffer); // Buffer overflow, discard oldest data
            }
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS)
            btHandle = param->start.handle;
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        btHandle = param->srv_open.handle;
        flag_write = true;
        break;
    case ESP_SPP_CLOSE_EVT:
        flag_write = false;
        btHandle = 0;
        break;
    case ESP_SPP_INIT_EVT:

        break;
    case ESP_SPP_CL_INIT_EVT:

        break;
    default:
        break;
    }
}

void SerialBT::bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    printf("gap %d\n", event);
    switch (event)
    {
    case ESP_BT_GAP_MODE_CHG_EVT: // Enum 13
        ESP_LOGI("GAP", "ESP_BT_GAP_MODE_CHG_EVT: mode: %d", param->mode_chg.mode);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        // esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 2, param->cfm_req.bda);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        esp_spp_disconnect(btHandle);
    default:
        break;
    }
    return;
}
