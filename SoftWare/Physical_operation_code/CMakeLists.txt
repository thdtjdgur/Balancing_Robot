idf_component_register(
                    SRCS "lidar.c" "hc06.c" "rx28.c" "pwm.c" "pid.c" "imu.c" "encoder.c" "app_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES 
                        esp_driver_gpio 
                        esp_driver_uart 
                        esp_driver_i2c 
                        esp_driver_mcpwm
                        esp_driver_spi
                        esp_driver_gptimer
                    REQUIRES 
                        esp_timer
                        bt
                        nvs_flash
 )
