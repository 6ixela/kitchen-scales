#ifndef BUTTON_H
#define BUTTON_H

#include <sys/lock.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


typedef struct button_t
{
    uint8_t id;
    uint8_t is_pressed;
    gpio_num_t pin;
    _lock_t mutex;
    QueueHandle_t msg_q_sensor;
} button_t;

void button_init(button_t *button, gpio_num_t pin, uint8_t id, QueueHandle_t msg_q_lcd);
uint8_t button_is_pressed(button_t *button);
void button_update_state(button_t *button);

#endif /* !BUTTON_H */