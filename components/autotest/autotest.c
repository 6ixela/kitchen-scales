#include "autotest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#define SECONDS (1000 / portTICK_PERIOD_MS)

static const char *TAG = "autotest";

static void startLedTest(autotest_t *componants)
{
    const char *lcd_screen = "Test des LEDs";
    const char *second_line = "D1 et D6";
    lcd_set_cursor(componants->lcd, 0, 0);
    lcd_print(componants->lcd, lcd_screen);
    lcd_set_cursor(componants->lcd, 1, 0);
    lcd_print(componants->lcd, second_line);
    lcd_set_cursor(componants->lcd, 0, 0);

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
    
    lcd_set_cursor(componants->lcd, 0, 0);
    lcd_print(componants->lcd, first_line);
    lcd_set_cursor(componants->lcd, 1, 0);
    vTaskDelay(2 * SECONDS);
    lcd_print(componants->lcd, second_line);
    vTaskDelay(2 * SECONDS);
    ESP_LOGI(TAG, "Ldc test");
}

static void startBuzzerTest(autotest_t *componants)
{
    buzzer_off(componants->buzzer);
    // buzzer_blink(componants->buzzer, 5, 500);
    ESP_LOGI(TAG, "Buzzer test");
}

static void ButtonTest(button_t *button)
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
    }
    else
    {
        ESP_LOGI(TAG, "Touche KO");
    }
}

static void startButtonTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Button1 test");
    ButtonTest(componants->button1);
    ESP_LOGI(TAG, "Button2 test");
    ButtonTest(componants->button2);
}

static void startPressureTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Pressure test");
    // fill les valeur pour avoir une valeur de base
    int raw = pressure_read_raw();
    float v = pressure_read_voltage();
    float p = pressure_read_pressure();

    printf("ADC=%d | V=%.2fV | P=%.1f kPa\n", raw, v, p);
    vTaskDelay(pdMS_TO_TICKS(1000));
    // TODO: timeout de 10sec
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
