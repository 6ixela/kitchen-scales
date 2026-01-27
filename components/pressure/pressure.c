#include "pressure.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../../main/message.h"

#define PRESSURE_ADC_CHANNEL ADC2_CHANNEL_2   // GPIO2
#define ADC_ATTEN            ADC_ATTEN_DB_11
#define ADC_WIDTH            ADC_WIDTH_BIT_12

static esp_adc_cal_characteristics_t adc_chars;

static int32_t values[10] = {0};
static size_t values_index = 0;
// Calibration data for weight sensor (logarithmic)
static float log_a = 0.0f;      // Logarithmic coefficient
static float log_b = 0.0f;      // Logarithmic offset
static bool is_calibrated = false;

void pressure_init(pressure_t *pressure, uint8_t id, QueueHandle_t msg_q_sensor)
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
    pressure->id = id;
    pressure->msg_q_sensor = msg_q_sensor;
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
    // Not used in logarithmic calibration
}

void pressure_calibrate(float v_100g, float v_1000g)
{
    // Logarithmic calibration: weight = a * ln(voltage) + b
    // 100 = a * ln(v_100g) + b
    // 1000 = a * ln(v_1000g) + b
    
    float ln_v_100g = logf(v_100g);
    float ln_v_1000g = logf(v_1000g);
    
    // a = (1000 - 100) / (ln(v_1000g) - ln(v_100g))
    float dW = 1000.0f - 100.0f;
    float d_ln_v = ln_v_1000g - ln_v_100g;
    
    if (d_ln_v != 0.0f)
    {
        log_a = dW / d_ln_v;
        log_b = 100.0f - (log_a * ln_v_100g);
        is_calibrated = true;
    }
}

float pressure_read_weight(void)
{
    float v = pressure_read_voltage();
    
    if (!is_calibrated || v <= 0.0f)
    {
        return 0.0f;
    }
    
    // weight = a * ln(v) + b
    float weight = log_a * logf(v) + log_b;
    
    if (weight < 0.0f)
        weight = 0.0f;
    if (weight > 5000.0f)
        weight = 5000.0f;
    return weight;
}

static int32_t mean(int32_t *arr, size_t len)
{
    int64_t sum = 0;
    for (size_t i = 0; i < len; i++)
    {
        sum += arr[i];
    }
    return (int32_t)(sum / len);
}

void pressure_task(void *args)
{
    pressure_t *pressure = (pressure_t *)args;
    msg_t msg;
    pressure_calibrate(2.7f, 0.83f);  // Example calibration
 
    while (1)
    {
        float weight = pressure_read_weight();
        values[values_index] = (int32_t)roundf(weight);
        values_index = (values_index + 1) % 10;
        msg.id = pressure->id;
        msg.value = mean(values, 10);
        printf("Voltage: %.3f V, Weight: %.2f g\n", pressure_read_voltage(), weight);
        if (values_index == 0)
        {
            xQueueSend(pressure->msg_q_sensor, &msg, portMAX_DELAY);
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}