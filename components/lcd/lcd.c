#include "lcd.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/lock.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define LCD_ADDR 0x3E

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

void lcd_cmd(uint8_t cmd)
{
    uint8_t data[2] = { 0x80, cmd };
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data, 2,
                               1000 / portTICK_PERIOD_MS);
}

void lcd_data(uint8_t data)
{
    uint8_t buf[2] = { 0x40, data };
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, buf, 2,
                               1000 / portTICK_PERIOD_MS);
}

void lcd_set_cursor(lcd_t *lcd, uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_cmd(addr);
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
    lcd_set_cursor(lcd, 0, 0);
}

void lcd_clear(lcd_t *lcd)
{
    _lock_acquire(&lcd->mutex);
    lcd_cmd(0x01); // Clear
    vTaskDelay(2 / portTICK_PERIOD_MS);
    _lock_release(&lcd->mutex);
}

void lcd_print(lcd_t *lcd, const char *str)
{
    _lock_acquire(&lcd->mutex);
    while (*str)
    {
        lcd_data(*str++);
    }
    _lock_release(&lcd->mutex);
}
