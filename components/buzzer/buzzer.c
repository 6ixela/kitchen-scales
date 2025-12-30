#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/lock.h>

#define OFF 0
#define ON 1

void buzzer_init(buzzer_t* buzzer, gpio_num_t pin)
{
    buzzer->pin = pin;
    buzzer->state = OFF;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    _lock_init(&buzzer->mutex);
}

void buzzer_on(buzzer_t* buzzer)
{
    _lock_acquire(&buzzer->mutex);
    buzzer->state = ON;
    gpio_set_level(buzzer->pin, ON);
    _lock_release(&buzzer->mutex);
}

void buzzer_off(buzzer_t* buzzer)
{
    _lock_acquire(&buzzer->mutex);
    buzzer->state = OFF;
    gpio_set_level(buzzer->pin, OFF);
    _lock_release(&buzzer->mutex);
}
