#include "pressure.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#define PRESSURE_ADC_CHANNEL ADC2_CHANNEL_2   // GPIO2
#define ADC_ATTEN            ADC_ATTEN_DB_11
#define ADC_WIDTH            ADC_WIDTH_BIT_12

static esp_adc_cal_characteristics_t adc_chars;

// Calibration data for weight sensor
static float weight_offset_voltage = 0.0f;  // Voltage at 0g
static float weight_sensitivity = 0.0f;    // V/g
static bool is_calibrated = false;

void pressure_init(uint8_t id, QueueHandle_t msg_q_lcd)
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

void pressure_tare(void)
{
    weight_offset_voltage = pressure_read_voltage();
}

void pressure_calibrate(float v_100g, float v_1000g)
{
    weight_offset_voltage = v_100g;

    float dV = v_1000g - v_100g;  // Typically negative (voltage decreases)
    float dW = 1000.0f - 100.0f;   // 900g difference
    
    if (dV != 0.0f)
    {
        weight_sensitivity = dW / dV;  // g/V
    }
    is_calibrated = true;
}

float pressure_read_weight(void)
{
    float v = pressure_read_voltage();
    
    if (!is_calibrated)
    {
        return 0.0f;
    }
    float dV = v - weight_offset_voltage;
    
    float weight = 100.0f + (weight_sensitivity * dV);
    
    if (weight < 0.0f)
        weight = 0.0f;
    if (weight > 5000.0f)
        weight = 5000.0f;
    return weight;
}
