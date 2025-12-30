#ifndef BUTTON_H
#define BUTTON_H

#include <driver/gpio.h>
#include <sys/lock.h>

typedef struct buzzer_t
{
    gpio_num_t pin;
    uint8_t state;
    _lock_t mutex;
} buzzer_t;

void buzzer_init(buzzer_t* buzzer, gpio_num_t pin);
void buzzer_on(buzzer_t* buzzer);
void buzzer_off(buzzer_t* buzzer);

#endif /* !BUTTON_H */