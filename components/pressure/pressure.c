#include "pressure.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../main/message.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define PRESSURE_ADC_CHANNEL ADC2_CHANNEL_2 // GPIO2
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12

static esp_adc_cal_characteristics_t adc_chars;

static int32_t values[10] = { 0 };
static size_t values_index = 0;

#define ADC_RESOLUTION_BITS 12
#define ADC_MAX_VALUE 4095.0
#define V_REF 5

// Caractéristiques du circuit et de la jauge
#define V_SUPPLY 3.3 // Tension d'alimentation du pont diviseur (V)
#define R_M 10000.0 // Résistance fixe Rm (10 kOhms)

// Équation : Rjauge = a * Force^(-b)
static float CALIBRATION_COEF_A = 10000.0;
static float CALIBRATION_COEF_B = 0.7;

static float CALIBRATION_OFFSET_G =
    0.0; // Mesure en grammes correspondant à 0g réel

static float last_value = 0.0f;

void pressure_init(pressure_t *pressure, uint8_t id, QueueHandle_t msg_q_sensor)
{
    adc1_config_width(ADC_WIDTH);

    adc2_config_channel_atten(PRESSURE_ADC_CHANNEL, ADC_ATTEN);

    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN, ADC_WIDTH,
                             5000, // Vref par défaut (mV)
                             &adc_chars);
    pressure->id = id;
    pressure->msg_q_sensor = msg_q_sensor;
}

int pressure_read_raw(void)
{
    int raw = 0;
    adc2_get_raw(PRESSURE_ADC_CHANNEL, ADC_WIDTH, &raw);
    return raw; // 0–4095
}

void pressure_tare(void)
{
    CALIBRATION_OFFSET_G = last_value;
}

/**
 * @brief Convertit une lecture brute du CAN en Tension mesurée (V_mesurée).
 *
 * @param adc_raw_value Valeur brute lue par le CAN (ex: 0 à 4095).
 * @return float Tension mesurée en Volts (V_mesurée).
 */
float convert_adc_to_voltage(int adc_raw_value)
{
    uint32_t mV = esp_adc_cal_raw_to_voltage(adc_raw_value, &adc_chars);
    return (float)mV / 1000.0;
}

/**
 * @brief Calcule la résistance de la jauge (Rjauge) à partir de la tension
 * mesurée.
 *
 * Formule inverse du pont diviseur de tension : Rjauge = (V_mesurée * R_M) /
 * (V_supply - V_mesurée)
 *
 * @param V_mesuree Tension mesurée en Volts.
 * @return float Résistance de la jauge en Ohms.
 */
float calculate_r_jauge(float V_mesuree)
{
    if (V_mesuree >= V_SUPPLY)
    {
        return R_M * 100.0; // Retourne une valeur très grande
    }
    return (V_mesuree * R_M) / (V_SUPPLY - V_mesuree);
}

/**
 * @brief Calcule la Force (Poids) en grammes à partir de la résistance de la
 * jauge.
 *
 * Formule inverse de l'équation de la jauge : Force = (a / Rjauge)^(1/b)
 *
 * @param R_jauge Résistance de la jauge en Ohms.
 * @return float Poids brut estimé en grammes.
 */
float calculate_force_from_r_jauge(float R_jauge)
{
    if (R_jauge == 0.0)
    {
        return 0.0;
    }
    float exponent = 1.0 / CALIBRATION_COEF_B;
    return powf((CALIBRATION_COEF_A / R_jauge), exponent);
}

/**
 * @brief Fonction principale pour obtenir le poids final après moyennage et
 * application de l'offset.
 *
 * @param current_adc_readings Tableau des 10 dernières lectures brutes du CAN.
 * @return float Poids final en grammes.
 */
float get_final_weight_g(int32_t current_adc_readings[])
{
    int i;
    long adc_sum = 0;

    for (i = 0; i < 10; i++)
    {
        adc_sum += current_adc_readings[i];
    }

    float adc_average = (float)adc_sum / 10.0;
    float V_mesuree = convert_adc_to_voltage((int)adc_average);
    printf("Measured Voltage1: %f mV\n", V_mesuree);
    float R_jauge = calculate_r_jauge(V_mesuree);

    float weight_brut = calculate_force_from_r_jauge(R_jauge);

    float final_weight = weight_brut * 100 - CALIBRATION_OFFSET_G;

    if (final_weight < 0.0)
    {
        final_weight = 0.0;
    }

    return final_weight;
}

void pressure_task(void *args)
{
    pressure_t *pressure = (pressure_t *)args;
    msg_t msg;

    while (1)
    {
        int32_t raw = pressure_read_raw();
        values[values_index] = raw;
        values_index = (values_index + 1) % 10;
        if (values_index == 0)
        {
            float weight = (float)get_final_weight_g(values);
            printf("Weight: %.2f g\n", weight);
            last_value = weight;
            msg.id = pressure->id;
            msg.value = (uint32_t)weight;
            xQueueSend(pressure->msg_q_sensor, &msg, portMAX_DELAY);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
