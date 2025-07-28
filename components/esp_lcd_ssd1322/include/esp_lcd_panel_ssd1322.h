/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_lcd_panel_dev.h"

#define SSD1322_PANEL_WIDTH 128
#define SSD1322_PANEL_HEIGHT 64

// SSD1322 commands
#define SSD1322_ENABLE_GRAYSCALE_TABLE         0x00 // No params
#define SSD1322_SET_COLUMN_ADDRESS             0x15 // 2 params, default 0, 119
#define SSD1322_WRITE_RAM                      0x5C // No params
#define SSD1322_READ_RAM                       0x5D // No params, not supported over SPI
#define SSD1322_SET_ROW_ADDRESS                0x75 // 2 params, default 0, 127
#define SSD1322_SET_REMAP                      0xA0 // 2 params, default 0x00, 0x01
#define SSD1322_SET_DISPLAY_START_LINE         0xA1 // 1 param, default 0
#define SSD1322_SET_DISPLAY_OFFSET             0xA2 // 1 param, default 0
#define SSD1322_ENTIRE_DISPLAY_OFF             0xA4 // No params
#define SSD1322_ENTIRE_DISPLAY_ON              0xA5 // No params
#define SSD1322_NORMAL_DISPLAY                 0xA6 // No params
#define SSD1322_INVERSE_DISPLAY                0xA7 // No params
#define SSD1322_ENABLE_PARTIAL_DISPLAY         0xA8 // 2 params, default 0, 127
#define SSD1322_EXIT_PARTIAL_DISPLAY           0xA9 // No params
#define SSD1322_FUNCTION_SELECTION             0xAB // 1 param, default 0x01
#define SSD1322_SLEEP_MODE_ON                  0xAE // No params
#define SSD1322_SLEEP_MODE_OFF                 0xAF // No params
#define SSD1322_SET_PHASE_LENGTH               0xB1 // 1 params, default 0x74
#define SSD1322_SET_DISPLAY_CLOCK_DIVIDER      0xB3 // (and osc. freq.) 1 param, default 0x50
#define SSD1322_DISPLAY_ENHANCE                0xB4 // 2 params, default 0xA2, 0xB5
#define SSD1322_SET_GPIO                       0xB5 // 1 param, default 0x0A
#define SSD1322_SET_SECOND_PRECHARGE_PERIOD    0xB6 // 1 param, default 0x08
#define SSD1322_SET_GRAYSCALE_TABLE            0xB8 // 15 params, default as set by B9 command
#define SSD1322_SELECT_DEFAULT_GRAYSCALE_TABLE 0xB9 // No params
#define SSD1322_SET_PRECHARGE_VOLTAGE          0xBB // 1 param, default 0x17
#define SSD1322_SET_VCOMH_VOLTAGE              0xBE // 1 param, default 0x04
#define SSD1322_SET_CONTRAST_CONTROL           0xC1 // 1 param, default 0x7F
#define SSD1322_MASTER_CONTRAST_CONTROL        0xC7 // 1 param, default 0x0F
#define SSD1322_SET_MULTIPLEX_RATIO            0xCA // 1 param, default 127
#define SSD1322_DISPLAY_ENHANCE_B              0xD1 // 2 params, default 0xA2, 0x20
#define SSD1322_SET_COMMAND_LOCK               0xFD // 1 param, default 0x12

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SSD1322 configuration structure
 *
 * To be used as esp_lcd_panel_dev_config_t.vendor_config.
 * See esp_lcd_new_panel_ssd1322().
 */
typedef struct {
    /* Nothing yet, reserved for future non-NHD-2.7-12864xxxx support */
} esp_lcd_panel_ssd1322_config_t;

/**
 * @brief Create LCD panel for model SSD1322
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 *
 * @note Only NHD-2.7-12864xxxx displays are supported currently.
 * Example usage:
 * @code {c}
 *
 * esp_lcd_panel_ssd1322_config_t ssd1322_config = {};
 * esp_lcd_panel_dev_config_t panel_config = {
 *     <...>
 *     .vendor_config = &ssd1322_config
 * };
 *
 * esp_lcd_panel_handle_t panel_handle = NULL;
 * esp_lcd_new_panel_ssd1322(io_handle, &panel_config, &panel_handle);
 * @endcode
 */
esp_err_t esp_lcd_new_panel_ssd1322(
    const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
    esp_lcd_panel_handle_t *ret_panel
);

/**
 * Non-standard additional function to set display contrast scale.
 */
esp_err_t panel_ssd1322_set_contrast(esp_lcd_panel_handle_t panel, uint8_t contrast);

#ifdef __cplusplus
}
#endif