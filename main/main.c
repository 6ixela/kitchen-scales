#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "lcd.h"
#include "led.h"
#include "buzzer.h"
#include "pressure.h"
#include "button.h"
#include "autotest.h"
#include "message.h"



void app_main(void)
{
    QueueHandle_t msg_q_value;
    QueueHandle_t msg_q_ldc;

    msg_q_value = xQueueCreate(20, sizeof(msg_t));
    msg_q_ldc = xQueueCreate(20, sizeof(msg_t));

    uint8_t id = 0;
    lcd_t lcd;
    lcd_init(&lcd, id++);
    button_t button1;
    button_init(&button1, GPIO_NUM_33, id++);
    button_t button2;
    button_init(&button2, GPIO_NUM_32, id++);
    led_t led1;
    led_init(&led1, GPIO_NUM_22, id++);
    led_t led2;
    led_init(&led2, GPIO_NUM_23, id++);
    buzzer_t buzzer;
    buzzer_init(&buzzer, GPIO_NUM_18, id++);
    pressure_t pressure_sensor;
    pressure_init(id++);

    autotest_t componants = {
        .button1 = &button1,
        .button2 = &button2,
        .buzzer = &buzzer,
        .led1 = &led1,
        .led2 = &led2,
        .lcd = &lcd,
        .pressure = &pressure_sensor,
    };


    while (1)

    {
        startAutoTest(&componants);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // while (1)
    // {
    //     // Update button state (debounced)
    //     button_update_state(&button);

    //     // If button is pressed, turn on buzzer and controlled LED
    //     if (button_is_pressed(&button))
    //     {
    //         led_on(&led_controlled);
    //     }
    //     else
    //     {
    //         led_off(&led_controlled);
    //     }

    //     // Toggle the blinking LED
    //     led_toggle(&led_blink);

    //     // Delay for 500ms (blink every 500ms on, 500ms off)
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
}

