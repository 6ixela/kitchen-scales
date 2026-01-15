#include "lcd.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/lock.h"

#define I2C_MASTER_SCL_IO 27
#define I2C_MASTER_SDA_IO 14
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define LCD_ADDR 0x3E
#define LCD_CMD 0x80
#define LCD_DATA 0x40

#define SECOND (1000 / portTICK_PERIOD_MS)

static char lcd_buffer[LCD_ROWS * LCD_COLS];

void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t lcd_write(uint8_t mode, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mode, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void lcd_cmd(uint8_t cmd)
{
    lcd_write(LCD_CMD, cmd);
}

void lcd_data(uint8_t data)
{
    lcd_write(LCD_DATA, data);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = { 0x00, LCD_DATA };
    lcd_cmd(LCD_CMD | (col + row_offsets[row]));
}

void lcd_init(lcd_t *lcd)
{
    _lock_init(&lcd->mutex);
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_cmd(0x38); // Function set
    lcd_cmd(0x39); // Extended instruction
    lcd_cmd(0x14);
    lcd_cmd(0x70);
    lcd_cmd(0x56);
    lcd_cmd(0x6C);
    vTaskDelay(pdMS_TO_TICKS(200));

    lcd_cmd(0x38);
    lcd_cmd(0x0C); // Display ON
    lcd_cmd(0x01); // Clear
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_clear(lcd_t *lcd)
{
    _lock_acquire(&lcd->mutex);
    memset(lcd_buffer, ' ', 32);
    _lock_release(&lcd->mutex);
}

static void fill_buffer(const char *str)
{
    for (size_t i = 0; i < 16; i++)
    {
        if (*str == 0 || *str == '\n')
        {
            lcd_buffer[i] = ' ';
        }
        else
        {
            lcd_buffer[i] = *str;
            str++;
        }
    }
    for (size_t i = 16; i < 32; i++)
    {
        if (*str == 0 || *str == '\n')
        {
            lcd_buffer[i] = ' ';
        }
        else
        {
            lcd_buffer[i] = *str;
            str++;
        }
    }
}

void lcd_print(lcd_t *lcd, const char *str)
{
    _lock_acquire(&lcd->mutex);
    fill_buffer(str);
    for (size_t i = 0; i < 16; i++)
    {
        lcd_data(lcd_buffer[i]);
    }
    lcd_set_cursor(lcd, 1, 0);
    for (size_t i = 16; i < 32; i++)
    {
        lcd_data(lcd_buffer[i]);
    }
    lcd_set_cursor(lcd, 0, 0);
    _lock_release(&lcd->mutex);
}

static void move_buffer(uint8_t move_top)
{
    if (move_top)
    {
        for (int i = 0; i < 15; i++)
        {
            lcd_buffer[i] = lcd_buffer[i + 1];
        }
    }
    for (size_t i = 31; i > 16; i--)
    {
        lcd_buffer[i] = lcd_buffer[i - 1];
    }
}

static void print_buff()
{
    for (size_t i = 0; i < 32; i++)
    {
        putchar(lcd_buffer[i]);
    }
    putchar('\n');
}

void lcd_defil_name(lcd_t *lcd, const char *top, const char *bottom)
{
    memset(lcd_buffer, ' ', LCD_COLS * LCD_ROWS);
    int16_t i = 15;
    int16_t j = 16;
    int16_t bottom_len = strlen(bottom);
    uint8_t move_top = 1;
    for (int16_t tmp = 0; tmp < bottom_len; tmp++)
    {
        move_buffer(move_top);
        if (tmp < 16)
        {
            lcd_buffer[i] = *top++;
        }
        move_top = tmp < 15;
        lcd_buffer[j] = bottom[bottom_len - tmp - 1];
        lcd_print(lcd, lcd_buffer);
        print_buff();
        vTaskDelay(2 * SECOND);
    }
    for (size_t i = 0; i < 16; i++)
    {
        move_buffer(move_top);
        lcd_buffer[j] = ' ';
        lcd_print(lcd, lcd_buffer);
        print_buff();
        vTaskDelay(2 * SECOND);
    }
}
