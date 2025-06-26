idf_component_register(
    SRCS "hello_world_main.c"
    PRIV_REQUIRES spi_flash esp_driver_gpio esp_lcd_ssd1322
    INCLUDE_DIRS "include"
)
