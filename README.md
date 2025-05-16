# ADS111x_ESP-IDF_9177

ADS111x analog to digital converter library for ESP-IDF. It uses ESP-IDF I2C version 2 driver.
The main.c file provide overview on how to use this library.

## List of accessible functions (see main/ADS111x.h for detailed information):
1. void ads111x_reset_config_reg(ads111x_cfg_t* device_cfg)
2. void ads111x_read_from_reg(ads111x_cfg_t* device_cfg, uint8_t device_reg)
3. esp_err_t ads111x_configure_address(ads111x_address_e addr, ads111x_cfg_t* device_cfg)
4. void ads111x_mux_config(uint8_t my_arg, ads111x_cfg_t* device_cfg, bool otf)
5. void ads111x_gain_amp(uint8_t my_arg, ads111x_cfg_t* device_cfg, bool otf)
6. void ads111x_operating_mode(bool my_arg, ads111x_cfg_t* device_cfg, bool otf)
7. void ads111x_data_rate(uint8_t my_arg, ads111x_cfg_t* device_cfg, bool otf)
8. void ads111x_comp_mode(bool my_arg, ads111x_cfg_t* device_cfg, bool otf)
9. void ads111x_comp_polarity(bool my_arg, ads111x_cfg_t* device_cfg, bool otf)
10. void ads111x_comp_latch(bool my_arg, ads111x_cfg_t* device_cfg, bool otf)
11. void ads111x_comp_queue(uint8_t my_arg, ads111x_cfg_t* device_cfg, bool otf)
12. esp_err_t initialize_ads111x(i2c_master_dev_handle_t ads111x_dev_handle, ads111x_cfg_t* device_cfg)
13. esp_err_t ads111x_measure_raw(ads111x_cfg_t* device_cfg, uint16_t* output_data)
14. esp_err_t ads111x_measure_voltage(ads111x_cfg_t* device_cfg, float* output_data)
15. esp_err_t ads111x_measure_raw_specific_channel(ads111x_cfg_t* device_cfg, uint8_t mux_setting, uint16_t* output_data)
16. esp_err_t ads111x_measure_voltage_specific_channel(ads111x_cfg_t* device_cfg, uint8_t mux_setting, float* output_data)
17. esp_err_t ads111x_measure_raw_sweep(ads111x_cfg_t* device_cfg, uint16_t* output_data)
18. esp_err_t ads111x_measure_voltage_sweep(ads111x_cfg_t* device_cfg, float* output_data)
19. esp_err_t ads111x_set_threshold_raw(ads111x_cfg_t* device_cfg)
20. esp_err_t ads111x_set_threshold_voltage(ads111x_cfg_t* device_cfg, float* low_thresh, float* high_thresh)
21. esp_err_t ads111x_reset_threshold(ads111x_cfg_t* device_cfg)
22. esp_err_t ads111x_alert_ready_pin(bool my_arg, ads111x_cfg_t* device_cfg)

## How to use
### A. Simple ADC single-ended/differential mode
1. Configure i2c_master_bus_config_t (ESP-IDF I2C)
2. Define i2c_master_bus_handle_t (ESP-IDF I2C) variable
3. Call function i2c_new_master_bus (ESP-IDF I2C) using proper arguments
4. Define ads111x_cfg_t variable
5. Call ads111x_reset_config_reg function to reset all ads111x_cfg_t member
6. Call ads111x_configure_address function to configure ADS111x I2C device address
7. Configure i2c_device_config_t (ESP-IDF I2C)
8. Define i2c_master_dev_handle_t (ESP-IDF I2C) variable
9. Call i2c_master_bus_add_device (ESP-IDF I2C) function
10. Modify ads111x_cfg_t based on your need
11. Call initialize_ads111x to flash the config register, low threshold (optional) and high threshold (optional) register
12. ADC ready to use
    
### B. ADC with comparator alert
1. The same step as A. above but dont forget to add your low threshold and high threshold value at step 10
2. Alert/ready pin will give alert signal based on comparator setting
   
### C. ADC with conversion ready pin interrupt
1. The same step as A.
2. After step 11, call ads111x_alert_ready_pin to configure alert/ready pin to issue an interrupt when ADS111x done converting data

