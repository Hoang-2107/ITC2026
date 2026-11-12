idf_component_register(SRCS "console.c" "settings.c" "main.c" "line_sensor.c" "motor.c" "pid.c" "hcsr05.c" "obstacle.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_usb_serial_jtag
                                esp_adc esp_timer nvs_flash)
