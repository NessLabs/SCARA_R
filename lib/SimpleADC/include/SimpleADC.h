#ifndef _SIMPLEADC_H
#define _SIMPLEADC_H

#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

enum type
{
    ADC_READ_RAW,
    ADC_READ_MV,
};

class SimpleADC
{
public:
    SimpleADC();
    ~SimpleADC();
    void setup(int io_num, adc_bitwidth_t width = ADC_BITWIDTH_12);
    int read(type mode);

private:
    static adc_oneshot_unit_handle_t _handle;
    static adc_cali_handle_t _cali_handle;
    adc_channel_t _channel;
};

#endif // _SIMPLEADC_H