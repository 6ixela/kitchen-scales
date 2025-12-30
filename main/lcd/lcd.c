#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "sys/lock.h"
#include "lcd.h"

// Delay in microseconds
#define LCD_DELAY_US(us) ets_delay_us(us)

// Commands
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

// Entry mode set flags
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Display control flags
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// Function set flags
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2_LINE 0x08
#define LCD_1_LINE 0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS 0x00

// Static functions
static void lcd_write_nibble(lcd_t *lcd, uint8_t nibble);
static void lcd_write_byte(lcd_t *lcd, uint8_t byte);
static void lcd_send_cmd(lcd_t *lcd, uint8_t cmd);
static void lcd_send_data(lcd_t *lcd, uint8_t data);

void lcd_init(lcd_t *lcd) {
    // Set GPIO directions
    gpio_set_direction(lcd->rs, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->en, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->d4, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->d5, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->d6, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->d7, GPIO_MODE_OUTPUT);

    // Initialize mutex
    _lock_init(&lcd->mutex);

    // Initialization sequence
    vTaskDelay(50 / portTICK_PERIOD_MS); // Wait for LCD to power up

    // Set RS low for commands
    gpio_set_level(lcd->rs, 0);

    // Send 0x03 three times
    lcd_write_nibble(lcd, 0x03);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    lcd_write_nibble(lcd, 0x03);
    LCD_DELAY_US(150);

    lcd_write_nibble(lcd, 0x03);
    LCD_DELAY_US(150);

    // Set to 4-bit mode
    lcd_write_nibble(lcd, 0x02);
    LCD_DELAY_US(150);

    // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_send_cmd(lcd, LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);

    // Display control: display on, cursor off, blink off
    lcd_send_cmd(lcd, LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

    // Clear display
    lcd_clear(lcd);

    // Entry mode set: increment, no shift
    lcd_send_cmd(lcd, LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
}

void lcd_clear(lcd_t *lcd) {
    _lock_acquire(&lcd->mutex);
    lcd_send_cmd(lcd, LCD_CMD_CLEAR_DISPLAY);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    _lock_release(&lcd->mutex);
}

void lcd_home(lcd_t *lcd) {
    _lock_acquire(&lcd->mutex);
    lcd_send_cmd(lcd, LCD_CMD_RETURN_HOME);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    _lock_release(&lcd->mutex);
}

void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col) {
    _lock_acquire(&lcd->mutex);
    uint8_t addr = col;
    if (row == 1) {
        addr += 0x40; // Second row starts at 0x40
    }
    lcd_send_cmd(lcd, LCD_CMD_SET_DDRAM_ADDR | addr);
    _lock_release(&lcd->mutex);
}

void lcd_print(lcd_t *lcd, const char *str) {
    _lock_acquire(&lcd->mutex);
    while (*str) {
        lcd_print_char(lcd, *str++);
    }
    _lock_release(&lcd->mutex);
}

void lcd_print_char(lcd_t *lcd, char c) {
    _lock_acquire(&lcd->mutex);
    lcd_send_data(lcd, (uint8_t)c);
    _lock_release(&lcd->mutex);
}

static void lcd_write_nibble(lcd_t *lcd, uint8_t nibble) {
    gpio_set_level(lcd->d4, (nibble >> 0) & 0x01);
    gpio_set_level(lcd->d5, (nibble >> 1) & 0x01);
    gpio_set_level(lcd->d6, (nibble >> 2) & 0x01);
    gpio_set_level(lcd->d7, (nibble >> 3) & 0x01);

    // Pulse EN
    gpio_set_level(lcd->en, 1);
    LCD_DELAY_US(1);
    gpio_set_level(lcd->en, 0);
    LCD_DELAY_US(100);
}

static void lcd_write_byte(lcd_t *lcd, uint8_t byte) {
    lcd_write_nibble(lcd, byte >> 4); // High nibble
    lcd_write_nibble(lcd, byte & 0x0F); // Low nibble
}

static void lcd_send_cmd(lcd_t *lcd, uint8_t cmd) {
    gpio_set_level(lcd->rs, 0); // Command mode
    lcd_write_byte(lcd, cmd);
}

static void lcd_send_data(lcd_t *lcd, uint8_t data) {
    gpio_set_level(lcd->rs, 1); // Data mode
    lcd_write_byte(lcd, data);
}
