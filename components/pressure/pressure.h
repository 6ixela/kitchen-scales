#ifndef PRESSURE_H
#define PRESSURE_H

#include <stdint.h>
#include "driver/gpio.h"

typedef struct pressure_t
{
    uint8_t id;
    gpio_num_t pin;
    uint32_t reset_val;
    _lock_t mutex;
} pressure_t;

void pressure_init(uint8_t id);
int pressure_read_raw(void);
float pressure_read_voltage(void);
float pressure_read_pressure(void);


#endif /* PRESSURE_H */