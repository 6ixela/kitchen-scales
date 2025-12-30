#ifndef LED_H
#define LED_H

#include <driver/gpio.h>
#include <sys/lock.h>

typedef struct led_t
{
    gpio_num_t pin;
    uint8_t state;
    _lock_t mutex;
} led_t;

void led_init(led_t* led, gpio_num_t pin);
void led_on(led_t* led);
void led_off(led_t* led);
void led_toggle(led_t* led);
void led_set_state(led_t* led, uint8_t state);
uint8_t led_get_state(led_t* led);
void led_blink(led_t* led, uint32_t times, uint32_t delay_ms);

#endif /* !LED_H */