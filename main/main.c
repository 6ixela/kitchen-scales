#include "autotest.h"
#include "button.h"
#include "buzzer.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lcd.h"
#include "led.h"
#include "message.h"
#include "pressure.h"

void app_main(void)
{
    QueueHandle_t msg_q_sensor;
    QueueHandle_t msg_q_lcd;

    msg_q_sensor = xQueueCreate(20, sizeof(msg_t));
    msg_q_lcd = xQueueCreate(20, sizeof(msg_t));

    uint8_t id = 0;
    lcd_t lcd;
    lcd_init(&lcd, id++, msg_q_lcd);
    button_t button1;
    button_init(&button1, GPIO_NUM_33, id++, msg_q_sensor);
    button_t button2;
    button_init(&button2, GPIO_NUM_32, id++, msg_q_sensor);
    led_t led1;
    led_init(&led1, GPIO_NUM_22, id++);
    led_t led2;
    led_init(&led2, GPIO_NUM_23, id++);
    buzzer_t buzzer;
    buzzer_init(&buzzer, GPIO_NUM_18, id++);
    pressure_t pressure_sensor;
    pressure_init(id++, msg_q_sensor);

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
