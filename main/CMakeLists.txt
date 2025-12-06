idf_component_register(
    SRCS "main.c" "monogram.c"
    PRIV_REQUIRES spi_flash esp_driver_gpio vfs usb esp_lcd_ssd1322 duppa_i2c_encoder duppa_rgb_led_ring_small
    INCLUDE_DIRS "include"
)
