#include "button.h"

#include <sys/lock.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void button_init(button_t *button, gpio_num_t pin, uint8_t id, QueueHandle_t msg_q_sensor)
{
    button->id = id;
    button->pin = pin;
    button->is_pressed = 0;
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    _lock_init(&button->mutex);
    button->msg_q_sensor = msg_q_sensor;
}

void button_update_state(button_t *button)
{
    _lock_acquire(&button->mutex);
    uint8_t read1 = gpio_get_level(button->pin);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    uint8_t read2 = gpio_get_level(button->pin);
    if (read1 == read2)
    {
        button->is_pressed = (read1 == 0) ? 1 : 0;
    }
    _lock_release(&button->mutex);
}

uint8_t button_is_pressed(button_t *button)
{
    button_update_state(button);
    uint8_t pressed = button->is_pressed;
    return pressed;
}