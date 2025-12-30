#ifndef LCD_H
#define LCD_H

#include "driver/gpio.h"
#include <sys/lock.h>

#define LCD_ROWS 2
#define LCD_COLS 16

// LCD 4 pins and 16x2 characters
typedef struct lcd_t
{
    gpio_num_t rs;
    gpio_num_t en;
    gpio_num_t d4;
    gpio_num_t d5;
    gpio_num_t d6;
    gpio_num_t d7;
    _lock_t mutex;
} lcd_t;

// Function prototypes
void lcd_init(lcd_t *lcd);
void lcd_clear(lcd_t *lcd);
void lcd_home(lcd_t *lcd);
void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col);
void lcd_print(lcd_t *lcd, const char *str);
void lcd_print_char(lcd_t *lcd, char c);

#endif /* LCD_H */