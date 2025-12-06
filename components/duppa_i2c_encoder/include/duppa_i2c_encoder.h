#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATUS_FLAG_BUTTON_UP = (1 << 0), 
    STATUS_FLAG_BUTTON_DOWN = (1 << 1),
    STATUS_FLAG_INCREMENT = (1 << 3),
    STATUS_FLAG_DECREMENT = (1 << 4),
    STATUS_FLAG_MAXIMUM = (1 << 5),
    STATUS_FLAG_MINIMUM = (1 << 6),
} i2c_encoder_status_flag_t;

typedef void *i2c_encoder_handle_t;

esp_err_t i2c_encoder_create(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, uint32_t scl_speed_hz, i2c_encoder_handle_t *handle_ret);

esp_err_t i2c_encoder_delete(i2c_encoder_handle_t handle);

esp_err_t i2c_encoder_init(i2c_encoder_handle_t handle, bool relative_mode, i2c_encoder_status_flag_t interrupts);

esp_err_t i2c_encoder_set_range(i2c_encoder_handle_t handle, int8_t min, int8_t max, int8_t step);

esp_err_t i2c_encoder_read_value(i2c_encoder_handle_t handle, int8_t *value);

esp_err_t i2c_encoder_set_led_color(i2c_encoder_handle_t handle, uint8_t r, uint8_t g, uint8_t b);

esp_err_t i2c_encoder_poll_status(i2c_encoder_handle_t handle, uint32_t *flags);

esp_err_t i2c_encoder_setup_interrupt(gpio_num_t intr_gpio, EventGroupHandle_t event_group, EventBits_t interrupt_bit);

#ifdef __cplusplus
}
#endif
