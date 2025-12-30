#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "button.h"
#include "led.h"

void app_main(void)
{
    // Initialize button on GPIO 0 (assuming pull-up, active low)
    button_t button;
    button_init(&button, GPIO_NUM_27);

    // Initialize LED controlled by buzzer on GPIO 32
    led_t led_controlled;
    led_init(&led_controlled, GPIO_NUM_32);

    // Initialize blinking LED on GPIO 33
    led_t led_blink;
    led_init(&led_blink, GPIO_NUM_33);

    while (1)
    {
        // Update button state (debounced)
        button_update_state(&button);

        // If button is pressed, turn on buzzer and controlled LED
        if (button_is_pressed(&button))
        {
            led_on(&led_controlled);
        }
        else
        {
            led_off(&led_controlled);
        }

        // Toggle the blinking LED
        led_toggle(&led_blink);

        // Delay for 500ms (blink every 500ms on, 500ms off)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
