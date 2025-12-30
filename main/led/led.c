#include "led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/lock.h>

#define OFF 0
#define ON 1

void led_init(led_t* led, gpio_num_t pin)
{
    led->pin = pin;
    led->state = OFF;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    _lock_init(&led->mutex);
}

void led_on(led_t* led)
{
    _lock_acquire(&led->mutex);
    led->state = ON;
    gpio_set_level(led->pin, ON);
    _lock_release(&led->mutex);
}

void led_off(led_t* led)
{
    _lock_acquire(&led->mutex);
    led->state = OFF;
    gpio_set_level(led->pin, OFF);
    _lock_release(&led->mutex);
}

void led_toggle(led_t* led)
{
    _lock_acquire(&led->mutex);
    led->state = !led->state;
    gpio_set_level(led->pin, led->state);
    _lock_release(&led->mutex);
}

void led_set_state(led_t* led, uint8_t state)
{
    _lock_acquire(&led->mutex);
    led->state = state;
    gpio_set_level(led->pin, state);
    _lock_release(&led->mutex);
}

uint8_t led_get_state(led_t* led)
{
    _lock_acquire(&led->mutex);
    uint8_t state = led->state;
    _lock_release(&led->mutex);
    return state;
}

void led_blink(led_t* led, uint32_t times, uint32_t delay_ms)
{
    for (uint32_t i = 0; i < times; i++) {
        led_toggle(led);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        led_toggle(led);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}
