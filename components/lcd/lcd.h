#ifndef LCD_H
#define LCD_H

#include <driver/gpio.h>
#include <sys/lock.h>
#include <stdint.h>


#define LCD_ROWS 2
#define LCD_COLS 16

typedef struct lcd_t
{
    uint8_t id;
    _lock_t mutex;
} lcd_t;

void i2c_init(void);
void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col);
void lcd_init(lcd_t *lcd, uint8_t id);
void lcd_clear(lcd_t *lcd);
void lcd_print(lcd_t *lcd, const char *str);
void lcd_defil_name(lcd_t *lcd, const char *top, const char *bottom);

#endif /* LCD_H */