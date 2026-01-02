#ifndef BUTTON_H
#define BUTTON_H

#include "driver/gpio.h"
#include <sys/lock.h>

typedef struct button_t
{
    gpio_num_t pin;
    uint8_t is_pressed;
    _lock_t mutex;
} button_t;

void button_init(button_t* button, gpio_num_t pin);
uint8_t button_is_pressed(button_t* button);
void button_update_state(button_t* button);

#endif /* !BUTTON_H */