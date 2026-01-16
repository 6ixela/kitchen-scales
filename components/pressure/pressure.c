#include "pressure.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define PRESSURE_ADC_CHANNEL ADC2_CHANNEL_2   // GPIO2
#define ADC_ATTEN            ADC_ATTEN_DB_11
#define ADC_WIDTH            ADC_WIDTH_BIT_12

static esp_adc_cal_characteristics_t adc_chars;

void pressure_init(uint8_t id)
{
    // Résolution ADC
    adc1_config_width(ADC_WIDTH);

    // Atténuation sur GPIO2 (ADC2)
    adc2_config_channel_atten(PRESSURE_ADC_CHANNEL, ADC_ATTEN);

    // Calibration legacy
    esp_adc_cal_characterize(
        ADC_UNIT_2,
        ADC_ATTEN,
        ADC_WIDTH,
        1100,        // Vref par défaut (mV)
        &adc_chars
    );
}

int pressure_read_raw(void)
{
    int raw = 0;
    adc2_get_raw(PRESSURE_ADC_CHANNEL, ADC_WIDTH, &raw);
    return raw; // 0–4095
}

float pressure_read_voltage(void)
{
    int raw = pressure_read_raw();
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
    return mv / 1000.0f;
}

float pressure_read_pressure(void)
{
    float v = pressure_read_voltage();

    if (v < 0.5f) v = 0.5f;
    if (v > 4.5f) v = 4.5f;

    return (v - 0.5f) * (100.0f / 4.0f); // kPa
}
