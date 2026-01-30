#ifndef BUZZER_H
#define BUZZER_H

#include <sys/lock.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


typedef struct buzzer_t
{
    uint8_t id;
    gpio_num_t pin;
    uint8_t state;
    _lock_t mutex;
} buzzer_t;

void buzzer_init(buzzer_t *buzzer, gpio_num_t pin, uint8_t id);
void buzzer_on(buzzer_t *buzzer);
void buzzer_off(buzzer_t *buzzer);
void buzzer_blink(buzzer_t *buzzer, uint32_t times, uint32_t delay_ms);

#endif /* !BUZZER_H */