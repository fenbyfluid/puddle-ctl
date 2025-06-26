idf_component_register(
    SRCS "esp_lcd_panel_ssd1322.c"
    REQUIRES esp_lcd
    PRIV_REQUIRES esp_driver_gpio
    INCLUDE_DIRS "include"
)
