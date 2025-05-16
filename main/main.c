/*
 * Test file for ADS111x 
 * Author: Shalihuddin Al Fatah
 */

#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ADS111x.h"

#define I2C_SPEED_HZ 100000
//#define SCL_WAIT_US  10

static const char *TAG = "Main: ";
esp_err_t err;

void app_main(void)
{
    // I2C master bus configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_26,
        .sda_io_num = GPIO_NUM_27,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    err = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    
    // ADS111x config structure
    ads111x_cfg_t my_ads111x_cfg;

    // Reset ADS111x config structure to default
    // ALWAYS CALL THIS TO MAKE SURE THE LIBRARY HAS THE SAME DEFAULT CONFIG AS THE DEVICE
    ads111x_reset_config_reg(&my_ads111x_cfg);

    // Set device address
    err = ads111x_configure_address(ADDRPIN_TO_GND, &my_ads111x_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);

    // ADS111x I2C configuration
    i2c_device_config_t i2C_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = (uint16_t) my_ads111x_cfg.device_addr,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    i2c_master_dev_handle_t device_handle;
    err = i2c_master_bus_add_device(bus_handle, &i2C_device_cfg, &device_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);

    // Connection test to the ADS111x
    err = i2c_master_probe(bus_handle, (uint16_t) my_ads111x_cfg.device_addr, 100);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);

    // ADS111x device configuration
    my_ads111x_cfg.mux_config = ADS111x_MUX_SNGL_AIN0_GND;
    my_ads111x_cfg.gain_amp = ADS111x_FSR_4V096;
    my_ads111x_cfg.data_rate = ADS111x_DR_8SPS;
    my_ads111x_cfg.operating_mode = ADS111x_SINGLE_SHOT;

    // Initialize device 
    // Flash the config register
    err = initialize_ads111x(device_handle, &my_ads111x_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);

    // Read config register, to make sure all configuration made are correct
    uint16_t config_reg = 0;
    err = ads111x_read_config_reg(&my_ads111x_cfg, &config_reg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "Config reg.: %d", config_reg);

    // ADC measure test (raw)
    uint16_t adc_raw = 0;
    err = ads111x_measure_raw(&my_ads111x_cfg, &adc_raw);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC raw ads111x_measure_raw: %d", adc_raw);

    // ADC measure test (voltage)
    float adc_value = 0.0;
    err = ads111x_measure_voltage(&my_ads111x_cfg, &adc_value);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC voltage ads111x_measure_voltage: %f", adc_value);

    // ADC measure test specific channel (raw)
    err = ads111x_measure_raw_specific_channel(&my_ads111x_cfg, ADS111x_MUX_SNGL_AIN2_GND, &adc_raw);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC raw ads111x_measure_raw_specific_channel: %d", adc_raw);

    // ADC measure test specific channel (voltage)
    err = ads111x_measure_voltage_specific_channel(&my_ads111x_cfg, ADS111x_MUX_SNGL_AIN3_GND, &adc_value);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC voltage ads111x_measure_voltage_specific_channel: %f", adc_value);

    // ADC measure sweep (raw)
    uint16_t adc_raw_arr[4];
    err = ads111x_measure_raw_sweep(&my_ads111x_cfg, adc_raw_arr);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC raw ads111x_measure_raw_sweep: %d %d %d %d", adc_raw_arr[0], adc_raw_arr[1], adc_raw_arr[2], adc_raw_arr[3]);

    // ADC measure sweep (voltage)
    float adc_value_arr[4];
    err = ads111x_measure_voltage_sweep(&my_ads111x_cfg, adc_value_arr);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "ADC voltage ads111x_measure_voltage_sweep: %f %f %f %f", adc_value_arr[0], adc_value_arr[1], adc_value_arr[2], adc_value_arr[3]);


    // Set threshold raw test
    my_ads111x_cfg.low_threshold = 25000;
    my_ads111x_cfg.high_threshold = 30000;
    err = ads111x_set_threshold_raw(&my_ads111x_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error ads111x_set_threshold_raw: %s (0x%x)", esp_err_to_name(err), err);
    
    // Verify register value
    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_LO_THRESH_REG);
    // Expected result 61 A8
    ESP_LOGI(TAG, "Low thresh register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_HI_THRESH_REG);
    // Expected result 75 30
    ESP_LOGI(TAG, "High thresh register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    err = ads111x_read_config_reg(&my_ads111x_cfg, &config_reg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "Config reg.: %d", config_reg);


    // Set threshold voltage test
    float low_voltage = 2.0;
    float high_voltage = 3.2;
    err = ads111x_set_threshold_voltage(&my_ads111x_cfg, &low_voltage, &high_voltage);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error ads111x_set_threshold_voltage: %s (0x%x)", esp_err_to_name(err), err);

    // Verify register value
    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_LO_THRESH_REG);
    // Expected result 3E 7F
    ESP_LOGI(TAG, "Low thresh f register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_HI_THRESH_REG);
    // Expected result 63 FF
    ESP_LOGI(TAG, "High thresh f register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    err = ads111x_read_config_reg(&my_ads111x_cfg, &config_reg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "Config reg.: %d", config_reg);


    // Reset thresh test
    err = ads111x_reset_threshold(&my_ads111x_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error ads111x_reset_threshold: %s (0x%x)", esp_err_to_name(err), err);

    // Verify register value
    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_LO_THRESH_REG);
    ESP_LOGI(TAG, "Default low thresh register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    ads111x_read_from_reg(&my_ads111x_cfg, ADS111x_HI_THRESH_REG);
    ESP_LOGI(TAG, "Default high thresh register: %x %x", my_ads111x_cfg.buffer[0], my_ads111x_cfg.buffer[1]);

    err = ads111x_read_config_reg(&my_ads111x_cfg, &config_reg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error: %s (0x%x)", esp_err_to_name(err), err);
    ESP_LOGI(TAG, "Config reg.: %d", config_reg);

    ESP_LOGI(TAG, "Done");
}
