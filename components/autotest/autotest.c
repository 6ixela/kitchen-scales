#include "autotest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#define SECONDS (1000 / portTICK_PERIOD_MS)

static const char *TAG = "autotest";

static void startLedTest(autotest_t *componants)
{
    const char *lcd_screen = "Test des LEDs\nD1 et D6";
    lcd_clear(componants->lcd);
    lcd_print(componants->lcd, lcd_screen);

    led_off(componants->led1);
    led_off(componants->led2);
    ESP_LOGI(TAG, "Led test");
    led_blink(componants->led1, 5, 500);
    vTaskDelay(1 * SECONDS);
    led_blink(componants->led2, 5, 500);
}

static void startLcdTest(autotest_t *componants)
{
    const char *first_line = "EPITA  2025/2026";
    const char *second_line = "VASSEUR,JOUY,OLIVER";
    lcd_defil_name(componants->lcd, first_line, second_line);
    ESP_LOGI(TAG, "Ldc test");
}

static void startBuzzerTest(autotest_t *componants)
{
    const char *lcd_screen = "Test du Buzzer";
    lcd_clear(componants->lcd);
    lcd_set_cursor(componants->lcd, 0, 0);
    lcd_print(componants->lcd, lcd_screen);
    buzzer_off(componants->buzzer);
    buzzer_blink(componants->buzzer, 5, 500);
    ESP_LOGI(TAG, "Buzzer test");
}

static void ButtonTest(autotest_t *componants, button_t *button)
{
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(10000);
    uint8_t is_pressed = 0;

    while ((xTaskGetTickCount() - start_time) < timeout)
    {
        if (button_is_pressed(button))
        {
            ESP_LOGI(TAG, "Button test");
            is_pressed = 1;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (is_pressed)
    {
        ESP_LOGI(TAG, "Touche OK");
        lcd_print_line(componants->lcd, "Touche OK       ", 1);
    }
    else
    {
        ESP_LOGI(TAG, "Touche KO");
        lcd_set_cursor(componants->lcd, 0, 0);
        lcd_print_line(componants->lcd, "Touche KO       ", 1);
    }
    button->is_pressed = 0;
}

static void startButtonTest(autotest_t *componants)
{
    const char *lcd_button = "Button1 test";
    lcd_print(componants->lcd, lcd_button);
    ButtonTest(componants, componants->button1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd_button = "Button2 test";
    lcd_print(componants->lcd, lcd_button);
    ButtonTest(componants, componants->button2);
}

static void startPressureTest(autotest_t *componants)
{
    const char *lcd_screen = "Test du Capteur\nde Poids";
    lcd_clear(componants->lcd);
    lcd_print(componants->lcd, lcd_screen);
    
    ESP_LOGI(TAG, "Weight sensor test - Taring...");
    pressure_tare();  // Zero reference at current weight
    vTaskDelay(pdMS_TO_TICKS(500));

    int32_t values[10] = { 0 };
    size_t values_index = 0;
    
    ESP_LOGI(TAG, "Reading weight...");
    for (size_t i = 0; i < 10; i++)
    {
        int32_t raw = pressure_read_raw();
        float v = convert_adc_to_voltage(raw);
        values[values_index] = raw;
        values_index = (values_index + 1) % 10;
        printf("ADC=%ld | V=%.2fV\n", raw, v);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    float weight = get_final_weight_g(values);
    char weight_str[32];
    snprintf(weight_str, sizeof(weight_str), "Poids:\n %.2f g", weight);
    lcd_print(componants->lcd, weight_str);
}

void startAutoTest(autotest_t *componants)
{
    startLcdTest(componants);
    vTaskDelay(2 * SECONDS);
    startLedTest(componants);
    vTaskDelay(2 * SECONDS);
    startBuzzerTest(componants);
    vTaskDelay(1 * SECONDS);
    startButtonTest(componants);
    vTaskDelay(2 * SECONDS);
    startPressureTest(componants);
    vTaskDelay(3 * SECONDS);
}
