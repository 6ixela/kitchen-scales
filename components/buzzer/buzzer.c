#include "buzzer.h"

#include <sys/lock.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OFF 0
#define ON 1

static const char *TAG = "Buzzer";

void buzzer_init(buzzer_t *buzzer, gpio_num_t pin, uint8_t id)
{
    buzzer->id = id;
    buzzer->pin = pin;
    buzzer->state = OFF;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    _lock_init(&buzzer->mutex);
}

void buzzer_on(buzzer_t *buzzer)
{
    _lock_acquire(&buzzer->mutex);
    buzzer->state = ON;
    gpio_set_level(buzzer->pin, ON);
    _lock_release(&buzzer->mutex);
}

void buzzer_off(buzzer_t *buzzer)
{
    _lock_acquire(&buzzer->mutex);
    buzzer->state = OFF;
    gpio_set_level(buzzer->pin, OFF);
    _lock_release(&buzzer->mutex);
}

static void buzzer_toggle(buzzer_t *buzzer)
{
    _lock_acquire(&buzzer->mutex);
    buzzer->state = !buzzer->state;
    gpio_set_level(buzzer->pin, buzzer->state);
    _lock_release(&buzzer->mutex);
}

void buzzer_blink(buzzer_t *buzzer, uint32_t times, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < times; i++)
    {
        ESP_LOGI(TAG, "ON");
        buzzer_toggle(buzzer);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "OFF");
        buzzer_toggle(buzzer);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}