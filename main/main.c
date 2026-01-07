#include "autotest.h"
#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "led.h"
#include "pressure.h"

void app_main(void)
{
    button_t button1;
    button_init(&button1, GPIO_NUM_33);
    button_t button2;
    button_init(&button2, GPIO_NUM_32);
    lcd_t lcd;
    lcd_init(&lcd);
    led_t led1;
    led_init(&led1, GPIO_NUM_22);
    led_t led2;
    led_init(&led2, GPIO_NUM_23);
    buzzer_t buzzer;
    buzzer_init(&buzzer, GPIO_NUM_18);
    pressure_t pressure_sensor;
    pressure_init();

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
