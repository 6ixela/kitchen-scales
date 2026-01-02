#include "autotest.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define SECONDS (1000 / portTICK_PERIOD_MS)


static const char *TAG = "autotest";

static void startLedTest(autotest_t *componants)
{
    ESP_LOGI(TAG, "Led test");
}

static void startLcdTest(autotest_t *componants)
{
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
