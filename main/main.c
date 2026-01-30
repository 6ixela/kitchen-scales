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
    pressure_t pressure_sensor;
    pressure_init(&pressure_sensor, id++, msg_q_sensor);
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

    lcd_t lcd;
    lcd_init(&lcd, id++, msg_q_lcd);

    autotest_t componants = {
        .button1 = &button1,
        .button2 = &button2,
        .buzzer = &buzzer,
        .led1 = &led1,
        .led2 = &led2,
        .lcd = &lcd,
        .pressure = &pressure_sensor,
    };

    startAutoTest(&componants);
    xTaskCreate(lcd_task, "lcd_task", 2048, &lcd, 5, NULL);
    xTaskCreate(button_task, "button1_task", 2048, &button1, 5, NULL);
    xTaskCreate(button_task, "button2_task", 2048, &button2, 5, NULL);
    xTaskCreate(pressure_task, "pressure_task", 2048, &pressure_sensor, 5, NULL);
    msg_t msg;
    uint8_t veille_state = 1;
    while (1)
    {
        xQueueReceive(msg_q_sensor, &msg, portMAX_DELAY);
        if (msg.id == button1.id)
        {
            if (msg.value == 0)
            {
                buzzer_blink(&buzzer, 1, 100);
                xQueueSend(msg_q_lcd, &msg, portMAX_DELAY);
                veille_state = !veille_state;
            }
            led_toggle(&led2);
            continue;
        }
        else if (msg.id == button2.id)
        {
            pressure_tare();
            buzzer_blink(&buzzer, 1, 100);
            led_toggle(&led1);
        }
        if (veille_state)
        {
            xQueueSend(msg_q_lcd, &msg, portMAX_DELAY);
        }
        
    }
}
