#ifndef AUTOTEST_H
#define AUTOTEST_H

#include "led.h"
#include "button.h"

#include "buzzer.h"

typedef struct autotest_t
{
    led_t led1;
    led_t led2;
    button_t button1;
    button_t button2;
} autotest_t;

void startAutoTest(autotest_t *componants);

#endif /* AUTOTEST_H */