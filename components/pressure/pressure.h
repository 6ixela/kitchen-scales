#ifndef PRESSURE_H
#define PRESSURE_H

#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


typedef struct pressure_t
{
    uint8_t id;
    gpio_num_t pin;
    uint32_t reset_val;
    _lock_t mutex;
    QueueHandle_t msg_q_sensor;
} pressure_t;

void pressure_init(pressure_t *pressure, uint8_t id, QueueHandle_t msg_q_sensor);
int pressure_read_raw(void);
float pressure_read_voltage(void);
float pressure_read_pressure(void);
float pressure_read_weight(void);
void pressure_calibrate(float v_50g, float v_100g, float v_500g, float v_1000g);
void pressure_tare(void);
void pressure_task(void *args);


#endif /* PRESSURE_H */