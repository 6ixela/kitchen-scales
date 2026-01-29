#include "pressure.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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
// static float log_a = 0.0f;      // Logarithmic coefficient
// static float log_b = 0.0f;      // Logarithmic offset
// static bool is_calibrated = false;

// Caractéristiques de l'ADC de l'ESP32 (à vérifier selon la configuration)
#define ADC_RESOLUTION_BITS 12      // Résolution typique (0 à 4095)
#define ADC_MAX_VALUE       4095.0  // 2^12 - 1
#define V_REF               5     // Tension de référence (V) du CAN (peut être différente si un atténuateur est utilisé)

// Caractéristiques du circuit et de la jauge
#define V_SUPPLY            4.8         // Tension d'alimentation du pont diviseur (V)
#define R_M                 10000.0  // Résistance fixe Rm (10 kOhms)

// Coefficients de la jauge (ajustés en calibration 4.4.1)
// Équation : Rjauge = a * Force^(-b)
static float CALIBRATION_COEF_A = 10000.0;
static float CALIBRATION_COEF_B = 0.7;

// Offset (enregistré en calibration 4.4.2 et stocké en mémoire non volatile)
static float CALIBRATION_OFFSET_G = 0.0; // Mesure en grammes correspondant à 0g réel

static float last_value = 0.0f;

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

// float pressure_read_voltage(void)
// {
//     int raw = pressure_read_raw();
//     uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
//     return mv / 1000.0f;
// }

void pressure_tare(void)
{
    CALIBRATION_OFFSET_G = last_value;
}

// void pressure_calibrate(float v_50g, float v_100g, float v_500g, float v_1000g)
// {
//     // Logarithmic calibration with 4 points using least squares
//     // weight = a * ln(voltage) + b

//     // Calibration points
//     float weights[4] = {50.0f, 100.0f, 500.0f, 1000.0f};
//     float voltages[4] = {v_50g, v_100g, v_500g, v_1000g};

//     float sum_ln_v = 0.0f;
//     float sum_w = 0.0f;
//     float sum_ln_v_sq = 0.0f;
//     float sum_w_ln_v = 0.0f;

//     // Calculate sums for least squares
//     for (int i = 0; i < 4; i++)
//     {
//         if (voltages[i] <= 0.0f)
//             return;

//         float ln_v = logf(voltages[i]);
//         sum_ln_v += ln_v;
//         sum_w += weights[i];
//         sum_ln_v_sq += ln_v * ln_v;
//         sum_w_ln_v += weights[i] * ln_v;
//     }

//     // Least squares formulas
//     float denominator = (4.0f * sum_ln_v_sq) - (sum_ln_v * sum_ln_v);

//     if (denominator != 0.0f)
//     {
//         log_a = ((4.0f * sum_w_ln_v) - (sum_w * sum_ln_v)) / denominator;
//         log_b = (sum_w - (log_a * sum_ln_v)) / 4.0f;
//         is_calibrated = true;
//     }
// }

// float pressure_read_weight(void)
// {
//     float v = pressure_read_voltage();
    
//     if (!is_calibrated || v <= 0.0f)
//     {
//         return 0.0f;
//     }
    
//     // weight = a * ln(v) + b
//     float weight = log_a * logf(v) + log_b - tare_value;
    
//     if (weight < 0.0f)
//         weight = 0.0f;
//     if (weight > 5000.0f)
//         weight = 5000.0f;
//     return weight;
// }

// static int32_t mean(int32_t *arr, size_t len)
// {
//     int64_t sum = 0;
//     for (size_t i = 0; i < len; i++)
//     {
//         sum += arr[i];
//     }
//     return (int32_t)(sum / len);
// }

// ====================================================================
// CONSTANTES DU SYSTEME ET COEFFICIENTS DE CALIBRATION
// Ces valeurs doivent être ajustées après la phase de calibration (Mode 4.4)
// ====================================================================


// ====================================================================
// FONCTIONS DE CALCUL
// ====================================================================

/**
 * @brief Convertit une lecture brute du CAN en Tension mesurée (V_mesurée).
 * 
 * @param adc_raw_value Valeur brute lue par le CAN (ex: 0 à 4095).
 * @return float Tension mesurée en Volts (V_mesurée).
 */
float convert_adc_to_voltage(int adc_raw_value) {
    // Conversion simple de la valeur brute en tension basée sur la résolution et V_REF
    // Note: Pour une implémentation réelle sur ESP32, il faudrait 
    // tenir compte de la non-linéarité du CAN et de l'atténuation.
    return ((float)adc_raw_value / ADC_MAX_VALUE) * V_REF;
}

/**
 * @brief Calcule la résistance de la jauge (Rjauge) à partir de la tension mesurée.
 * 
 * Formule inverse du pont diviseur de tension : Rjauge = (V_mesurée * R_M) / (V_supply - V_mesurée)
 * 
 * @param V_mesuree Tension mesurée en Volts.
 * @return float Résistance de la jauge en Ohms.
 */
float calculate_r_jauge(float V_mesuree) {
    if (V_mesuree >= V_SUPPLY) {
        // Protection contre la division par zéro ou valeurs impossibles
        return R_M * 100.0; // Retourne une valeur très grande
    }
    return (V_mesuree * R_M) / (V_SUPPLY - V_mesuree);
}

/**
 * @brief Calcule la Force (Poids) en grammes à partir de la résistance de la jauge.
 * 
 * Formule inverse de l'équation de la jauge : Force = (a / Rjauge)^(1/b)
 * 
 * @param R_jauge Résistance de la jauge en Ohms.
 * @return float Poids brut estimé en grammes.
 */
float calculate_force_from_r_jauge(float R_jauge) {
    if (R_jauge == 0.0) {
        return 0.0;
    }
    // Calcul de 1/b
    float exponent = 1.0 / CALIBRATION_COEF_B;
    // Utilisation de la fonction pow() pour l'exponentiation
    return powf((CALIBRATION_COEF_A / R_jauge), exponent);
}


/**
 * @brief Fonction principale pour obtenir le poids final après moyennage et application de l'offset.
 * 
 * @param current_adc_readings Tableau des 10 dernières lectures brutes du CAN.
 * @return float Poids final en grammes.
 */
float get_final_weight_g(int32_t current_adc_readings[]) {
    int i;
    long adc_sum = 0;
    
    // 1. MOYENNAGE DES 10 MESURES
    for (i = 0; i < 10; i++) {
        adc_sum += current_adc_readings[i];
    }
    
    // Valeur brute moyenne du CAN
    float adc_average = (float)adc_sum / 10.0;
    
    // 2. CONVERSION ET CALCUL
    
    // a. Conversion de l'ADC vers la Tension
    float V_mesuree = convert_adc_to_voltage((int)adc_average);
    
    // b. Calcul de la Résistance de Jauge
    float R_jauge = calculate_r_jauge(V_mesuree);
    
    // c. Calcul du Poids Brut
    float weight_brut = calculate_force_from_r_jauge(R_jauge);
    
    // 3. APPLICATION DE L'OFFSET
    // Si la valeur est négative après soustraction de l'offset, on suppose 0g.
    float final_weight = weight_brut * 100 - CALIBRATION_OFFSET_G;
    
    if (final_weight < 0.0) {
        final_weight = 0.0;
    }
    
    return final_weight;
}

// ====================================================================
// EXEMPLE D'UTILISATION (Simulation)
// ====================================================================

// int main() {
//     // NOTE: Dans l'application réelle ESP32, vous liriez l'ADC ici
//     // et récupéreriez l'OFFSET de la mémoire non volatile.

//     // Simulation de 10 lectures brutes du CAN (correspondant à un poids X)
//     int adc_readings_sample[10] = {3100, 3105, 3098, 3103, 3101, 3100, 3102, 3104, 3099, 3100};
    
//     // Exemple de valeur d'OFFSET enregistrée (pour simuler la mesure à vide de 12g)
//     // Cette valeur devrait être calculée par la fonction calculate_force_from_r_jauge(R_jauge_a_vide)
//     CALIBRATION_OFFSET_G = 12.0; 
    
//     float weight = get_final_weight_g(adc_readings_sample);
    
//     printf("--- SIMULATION DE CALCUL DE PESEE ---\n");
//     printf("Offset applique (g) : %.2f\n", CALIBRATION_OFFSET_G);
//     printf("Valeur moyenne ADC : %.2f\n", (float)(3100+3105+3098+3103+3101+3100+3102+3104+3099+3100)/10.0);
//     printf("Poids final estime (g): %.2f\n", weight);
//     printf("--------------------------------------\n");

//     // Exemple de lecture à vide (avec un ADC moyen de 3150 correspondant à 12g brut)
//     int adc_readings_zero[10] = {3150, 3150, 3150, 3150, 3150, 3150, 3150, 3150, 3150, 3150};
//     float weight_zero = get_final_weight_g(adc_readings_zero);
//     printf("Poids estime a vide (doit etre proche de 0g): %.2f\n", weight_zero);
    
//     return 0;
// }

void pressure_task(void *args)
{
    pressure_t *pressure = (pressure_t *)args;
    msg_t msg;
    // pressure_calibrate(3.0f, 2.3f, 1.0f, 0.83f);  // Example calibration
 
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
