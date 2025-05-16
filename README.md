# ADS111x_ESP-IDF_9177

ADS111x analog to digital converter library for ESP-IDF. It uses ESP-IDF I2C version 2 driver.

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

## Device configuration structure
typedef struct {
   /* ---- Managed by the library ---- */
   uint8_t device_addr;    
   uint8_t buffer[3];        // I2C communication buffer
   float PGA_float;          // Gain amplifier value in float
   uint8_t MSB_config_data; 
   uint8_t LSB_config_data;
   esp_err_t ADS111x_err;    // Error code

   /* ---- Managed by user ---- */
   uint8_t mux_config;
   uint8_t gain_amp;         // Gain amplifier register config
   bool operating_mode;
   uint8_t data_rate;
   bool comp_mode;           // Comparator mode
   bool comp_pol;            // Comparator polarity
   bool comp_latch;          // Comparator latching
   uint8_t comp_queue;       // Comparator queue
   uint16_t low_threshold;   // Low threshold register 
   uint16_t high_threshold;  // High threshold register
   i2c_master_dev_handle_t ads111x_i2c_dev_handle; 
} ads111x_cfg_t;

