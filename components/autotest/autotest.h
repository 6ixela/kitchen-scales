#ifndef AUTOTEST_H
#define AUTOTEST_H

#include "button.h"
#include "buzzer.h"
#include "led.h"
#include "lcd.h"

typedef struct autotest_t
{
    led_t *led1;
    led_t *led2;
    buzzer_t *buzzer;
    button_t *button1;
    button_t *button2;
    lcd_t *lcd;
} autotest_t;

void startAutoTest(autotest_t *componants);

#endif /* AUTOTEST_H */