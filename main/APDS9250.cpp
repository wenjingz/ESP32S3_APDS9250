#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <APDS9250.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define SDA_PIN 8
#define SCL_PIN 9

static i2c_port_t i2c_port = I2C_NUM_0;

extern "C"
{
    void app_main(void);
}

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    conf.master.clk_speed = 1000000;

    return i2c_param_config(i2c_port, &conf);
}

int8_t APDS9250_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = 0;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0;
    }
    else
    {
        iError = -1;
    }

    i2c_cmd_link_delete(cmd);
    // ESP_LOGI("APDS", "reg_data = %u", *reg_data);
    return (uint8_t)iError;
}

int8_t APDS9250_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t iError = 0;

    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_write(cmd, reg_data, cnt, true);

    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = 0;
        // ESP_LOGI("APDL9250", "Register writen success %d", iError);
    }
    else
    {
        iError = -1;
        // ESP_LOGE("APDL9250", "Register writen fail %d", iError);
    }
    i2c_cmd_link_delete(cmd);

    return (int8_t)iError;
}

void APDS9250_set_mode(uint8_t main_ctrl)
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_MAIN_CTRL;
    uint8_t *reg_data = &main_ctrl;
    uint8_t cnt = 1;
    APDS9250_write_reg(dev_addr, reg_addr, reg_data, cnt);
}

// Set measurement rate
void APDS9250_set_measure_rate(uint8_t measure_rate)
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_MEAS_RATE;
    uint8_t *reg_data = &measure_rate;
    uint8_t cnt = 1;
    APDS9250_write_reg(dev_addr, reg_addr, reg_data, cnt);
}

// set LS gain
void APDS9250_set_ls_gain(uint8_t gain_level)
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_GAIN;
    uint8_t *reg_data = &gain_level;
    uint8_t cnt = 1;
    APDS9250_write_reg(dev_addr, reg_addr, reg_data, cnt);
}

// transform three uint_8 to one 24bits uint32_t
uint32_t APDS9250_regTo24bits(uint8_t *values)
{
    uint32_t val;

    val = values[0];
    val |= values[1] << 8;
    val |= values[2] << 16;

    return val & 0x0FFFFF;
}

// get ALS value
uint32_t APDS9250_get_als()
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_DATA_GREEN_0;
    uint8_t count = 3;
    uint8_t values[count];
    APDS9250_read_reg(dev_addr, reg_addr, values, count);
    return APDS9250_regTo24bits(values);
}

// get red value
uint32_t APDS9250_get_red()
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_DATA_RED_0;
    uint8_t count = 3;
    uint8_t values[count];
    APDS9250_read_reg(dev_addr, reg_addr, values, count);
    return APDS9250_regTo24bits(values);
}

// get green value
uint32_t APDS9250_get_green()
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_DATA_GREEN_0;
    uint8_t count = 3;
    uint8_t values[count];
    APDS9250_read_reg(dev_addr, reg_addr, values, count);
    return APDS9250_regTo24bits(values);
}


// get blue value


uint32_t APDS9250_get_blue()
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_DATA_BLUE_0;
    uint8_t count = 3;
    uint8_t values[count];
    APDS9250_read_reg(dev_addr, reg_addr, values, count);
    return APDS9250_regTo24bits(values);
}

//get infra-red value

uint32_t APDS9250_get_IR()
{
    uint8_t dev_addr = APDS9250_ADDRESS;
    uint8_t reg_addr = APDS9250_REG_LS_DATA_IR_0;
    uint8_t count = 3;
    uint8_t values[count];
    APDS9250_read_reg(dev_addr, reg_addr, values, count);
    return APDS9250_regTo24bits(values);
}





void app_main(void)
{
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_master_driver_initialize();

    // int8_t error = APDS9250_write_reg(dev_addr, reg_addr, &APDS9250_ALS_MODE, cnt);

    APDS9250_set_mode(APDS9250_ALS_MODE);
    // Set measurement rate
    APDS9250_set_measure_rate(APDS9250_MEASURE_RATE_DEFAUT);
    // set ls gain level
    APDS9250_set_ls_gain(APDS9250_LS_GAIN_18);

    // get ALS data

    while (1)
    {
        uint32_t LUX = APDS9250_get_als();
        ESP_LOGI("APDL9250", "LUX = %d", LUX);
        // ESP_LOGI("APDL9250", "LUX = %d", LUX * 35 / 3 / 100 / 7);
        uint32_t IR = APDS9250_get_IR();
        ESP_LOGI("APDL9250", "IR = %d",IR);
        APDS9250_set_mode(APDS9250_CS_MODE);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        uint32_t red = APDS9250_get_red();
         ESP_LOGI("APDL9250", "RED = %d",red);
        uint32_t green = APDS9250_get_green();
         ESP_LOGI("APDL9250", "GREEN = %d",green);
        uint32_t blue = APDS9250_get_blue();
         ESP_LOGI("APDL9250", "BLUE= %d",blue);
        APDS9250_set_mode(APDS9250_ALS_MODE);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
