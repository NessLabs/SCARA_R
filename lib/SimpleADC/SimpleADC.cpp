#include "SimpleADC.h"

adc_oneshot_unit_handle_t SimpleADC::_handle = nullptr;
adc_cali_handle_t SimpleADC::_cali_handle = nullptr;

SimpleADC::SimpleADC()
{
}

SimpleADC::~SimpleADC()
{
    adc_oneshot_del_unit(_handle);
}

void SimpleADC::setup(int io_num, adc_bitwidth_t width)
{
    adc_unit_t unit;
    adc_oneshot_io_to_channel(io_num, &unit, &_channel);
    if (_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = unit,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        adc_oneshot_new_unit(&init_config, &_handle);
    }
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12, // Configure for 0-3.6V range
        .bitwidth = width,
    };
    adc_oneshot_config_channel(_handle, _channel, &chan_config);
    if (_cali_handle == nullptr) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = width,
            .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF,
        };
        adc_cali_create_scheme_line_fitting(&cali_config, &_cali_handle);
    }
}

int SimpleADC::read(type mode)
{
    int reading;
    if (mode == ADC_READ_RAW)
        ESP_ERROR_CHECK(adc_oneshot_read(_handle, _channel, &reading));
    else
        adc_oneshot_get_calibrated_result(_handle, _cali_handle, _channel, &reading);
    return reading;
}
