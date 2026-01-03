#include "autotest.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SECONDS (1000 / portTICK_PERIOD_MS)

static const char *TAG = "autotest";

static void startLedTest(autotest_t *componants)
{
    const char* lcd_screen = "Test des LEDs";
    const char* second_line = "D1 et D6";
    
    // TODO
    // lcd_print(componants->lcd, NULL);

    ESP_LOGI(TAG, "Led test");
    led_blink(componants->led1, 5, 500);
    vTaskDelay(1 * SECONDS);
    led_blink(componants->led1, 5, 500);

}

static void startLcdTest(autotest_t *componants)
{
    const char* first_line = "EPITA  2025/2026";
    const char* second_line = "VASSEUR,JOUY,OLIVER";
    //
    ESP_LOGI(TAG, "Ldc test");
}

static void startBuzzerTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Buzzer test");
}

static void startButtonTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Button test");
    // TODO: timeout de 10sec
}

static void startPressureTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Pressure test");
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
