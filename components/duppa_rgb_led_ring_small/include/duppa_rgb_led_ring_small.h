#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#define IS31FL3746A_ADDR1_GND 0b1100000
#define IS31FL3746A_ADDR1_SCL 0b1100001
#define IS31FL3746A_ADDR1_SDA 0b1100010
#define IS31FL3746A_ADDR1_VCC 0b1100011

#define IS31FL3746A_ADDR2_GND 0b1100000
#define IS31FL3746A_ADDR2_SCL 0b1100100
#define IS31FL3746A_ADDR2_SDA 0b1101000
#define IS31FL3746A_ADDR2_VCC 0b1101100

#define IS31FL3746A_LED_COUNT 24

#ifdef __cplusplus
extern "C" {
#endif

typedef void *is31fl3746a_handle_t;

esp_err_t is31fl3746a_create(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, uint32_t scl_speed_hz, is31fl3746a_handle_t *handle_ret);

esp_err_t is31fl3746a_delete(is31fl3746a_handle_t handle);

esp_err_t is31fl3746a_init(is31fl3746a_handle_t handle);

esp_err_t is31fl3746a_set_global_scale(is31fl3746a_handle_t handle, uint8_t scale);

esp_err_t is31fl3746a_set_led_scale(is31fl3746a_handle_t handle, uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);

esp_err_t is31fl3746a_flush_led_scale(is31fl3746a_handle_t handle);

esp_err_t is31fl3746a_set_led_color(is31fl3746a_handle_t handle, uint8_t led_index, uint8_t r, uint8_t g, uint8_t b);

esp_err_t is31fl3746a_flush_led_color(is31fl3746a_handle_t handle);

#ifdef __cplusplus
}
#endif
