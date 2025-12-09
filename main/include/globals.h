#pragma once

#include <inttypes.h>

#include "duppa_i2c_encoder.h"
#include "duppa_rgb_led_ring_small.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"

#define DISPLAY_COUNT 2
#define ENCODER_COUNT 4

extern SemaphoreHandle_t lvgl_mutex;

extern lv_display_t *display_handles[DISPLAY_COUNT];
extern lv_indev_t *encoder_indevs[ENCODER_COUNT];
extern i2c_encoder_handle_t encoder_handles[ENCODER_COUNT];
extern is31fl3746a_handle_t ring_handles[ENCODER_COUNT];