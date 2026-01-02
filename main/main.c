#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "button.h"
#include "led.h"
#include "autotest.h"

void app_main(void)
{
    // Initialize button on GPIO 0 (assuming pull-up, active low)
    button_t button1;
    button_init(&button1, GPIO_NUM_27);
    button_t button2;
    button_init(&button2, GPIO_NUM_28);

    // Initialize LED controlled by buzzer on GPIO 32
    led_t led1;
    led_init(&led1, GPIO_NUM_32);

    // Initialize blinking LED on GPIO 33
    led_t led2;
    led_init(&led2, GPIO_NUM_33);

    autotest_t componants = {
        .button1 = button1,
        .button2 = button2,
        .led1 = led1,
        .led2 = led2,
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
