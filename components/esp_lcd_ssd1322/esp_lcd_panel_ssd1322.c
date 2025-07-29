/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_lcd_panel_ssd1322.h"

#include <stdint.h>
#include <stdlib.h>
#include <sys/cdefs.h>

#include "sdkconfig.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_compiler.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lcd_panel.ssd1322";

static esp_err_t panel_ssd1322_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1322_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1322_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1322_draw_bitmap(
    esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data
);
static esp_err_t panel_ssd1322_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1322_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1322_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_ssd1322_disp_sleep(esp_lcd_panel_t *panel, bool sleep);

static const size_t SCREEN_BUFFER_SIZE = SSD1322_PANEL_WIDTH * SSD1322_PANEL_HEIGHT;

#define SSD1322_REMAP_FLAG_COLUMN_ORDER (1U << 1)
#define SSD1322_REMAP_FLAG_NIBBLE_ORDER (1U << 2)
#define SSD1322_REMAP_FLAG_ROW_ORDER    (1U << 4)

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    bool is_display_blanked;
    bool is_display_inverted;
    uint8_t *screen_buffer;
} ssd1322_panel_t;

esp_err_t esp_lcd_new_panel_ssd1322(
    const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
    esp_lcd_panel_handle_t *ret_panel
) {
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif

    esp_err_t ret = ESP_OK;
    ssd1322_panel_t *ssd1322 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

    // The controller is 4bpp, but the display needs each nibble to be doubled - we convert internally
    ESP_GOTO_ON_FALSE(panel_dev_config->bits_per_pixel == 8, ESP_ERR_INVALID_ARG, err, TAG, "bpp must be 8");

    esp_lcd_panel_ssd1322_config_t *ssd1322_spec_config =
        (esp_lcd_panel_ssd1322_config_t *)panel_dev_config->vendor_config;
    (void)ssd1322_spec_config; // Not used yet.

    // leak detection of ssd1322 because saving ssd1322->base address
    ESP_COMPILER_DIAGNOSTIC_PUSH_IGNORE("-Wanalyzer-malloc-leak")
    ssd1322 = calloc(1, sizeof(ssd1322_panel_t));
    ESP_GOTO_ON_FALSE(ssd1322, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1322 panel");

    // TODO: Make this an option, and an option to allocate in PSRAM
    // TODO: spi_bus_dma_memory_alloc ?
    ssd1322->screen_buffer = heap_caps_calloc(1, SCREEN_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(ssd1322->screen_buffer, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1322 screen buffer");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };

        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for reset line failed");
    }

    ssd1322->io = io;
    ssd1322->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1322->reset_level = panel_dev_config->flags.reset_active_high;
    ssd1322->base.reset = panel_ssd1322_reset;
    ssd1322->base.init = panel_ssd1322_init;
    ssd1322->base.del = panel_ssd1322_del;
    ssd1322->base.draw_bitmap = panel_ssd1322_draw_bitmap;
    ssd1322->base.mirror = panel_ssd1322_mirror;
    ssd1322->base.swap_xy = NULL; // Not supported in HW due to memory layout
    ssd1322->base.set_gap = NULL; // Doesn't make sense for an OLED
    ssd1322->base.invert_color = panel_ssd1322_invert_color;
    ssd1322->base.disp_on_off = panel_ssd1322_disp_on_off;
    ssd1322->base.disp_sleep = panel_ssd1322_disp_sleep;
    *ret_panel = &(ssd1322->base);
    ESP_LOGD(TAG, "new ssd1322 panel @%p", ssd1322);

    return ESP_OK;

err:
    if (ssd1322) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }

        if (ssd1322->screen_buffer) {
            free(ssd1322->screen_buffer);
            ssd1322->screen_buffer = NULL;
        }

        free(ssd1322);
    }

    return ret;

    ESP_COMPILER_DIAGNOSTIC_POP("-Wanalyzer-malloc-leak")
}

/**
 * Non-standard additional function to set display contrast scale.
 */
esp_err_t panel_ssd1322_set_contrast(esp_lcd_panel_t *panel, uint8_t contrast) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_CONTRAST_CONTROL,
            (uint8_t[]){
                contrast,
            },
            1
        ),
        TAG, "io tx param SSD1322_SET_CONTRAST_CONTROL failed"
    );

    return ESP_OK;
}

static esp_err_t panel_ssd1322_del(esp_lcd_panel_t *panel) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);

    if (ssd1322->reset_gpio_num >= 0) {
        gpio_reset_pin(ssd1322->reset_gpio_num);
    }

    if (ssd1322->screen_buffer) {
        free(ssd1322->screen_buffer);
        ssd1322->screen_buffer = NULL;
    }

    ESP_LOGD(TAG, "del ssd1322 panel @%p", ssd1322);
    free(ssd1322);

    return ESP_OK;
}

static esp_err_t panel_ssd1322_reset(esp_lcd_panel_t *panel) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);

    if (ssd1322->reset_gpio_num >= 0) {
        gpio_set_level(ssd1322->reset_gpio_num, ssd1322->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(ssd1322->reset_gpio_num, !ssd1322->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}

static esp_err_t panel_ssd1322_init(esp_lcd_panel_t *panel) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    // Init sequence from the NHD example code, with redundant commands removed:
    //   0xB3 0x91      Set Display Clock / Oscillator Frequency
    //   0xCA 0x3F      Set Multiplex Ratio
    //   0xA0 0x16 0x11 Set Remap. (A[1]) & (A[4]) can be adjusted to flip display orientation
    //   0xC1 0x9F      Set Contrast Control
    //   0xB1 0x72      Set Phase Length
    //   0xBB 0x1F      Set Precharge Voltage
    //   0xB4 0xA0 0xFD Enable External VSL
    //   0xB5 0x00      Set GPIO <- not a no-op, but unwanted
    //   0xAF           Display ON

    ESP_LOGD(TAG, "panel init");

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_DISPLAY_CLOCK_DIVIDER,
            (uint8_t[]){
                // Default 0x50 (freq 5, div 1), screen datasheet suggests 0x91 (freq 9, div 2)
                // 0x11 (freq 1, div 2) gets us reduced power consumption with no visible flicker.
                // 0x12 (freq 1, div 4) has a subtle whole-screen flicker - could investigate options between them.
                0x50,
            },
            1
        ),
        TAG, "io tx param SSD1322_SET_DISPLAY_CLOCK_DIVIDER failed"
    );

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_MULTIPLEX_RATIO,
            (uint8_t[]){
                // Display height
                SSD1322_PANEL_HEIGHT - 1,
            },
            1
        ),
        TAG, "io tx param SSD1322_SET_MULTIPLEX_RATIO failed"
    );

    ESP_RETURN_ON_ERROR(panel_ssd1322_mirror(panel, true, true), TAG, "panel_ssd1322_mirror failed");

    // Default is 127 out of 255, this increases it to 159
    ESP_RETURN_ON_ERROR(panel_ssd1322_set_contrast(panel, 0x9F), TAG, "panel_ssd1322_set_contrast failed");

    // Commenting this out doesn't seem to make any visible difference. Maybe a slight flicker.
    // According to an SSD app note, the banding we're seeing is likely incorrect pre-charging.
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_PHASE_LENGTH,
            (uint8_t[]){
                // Phase 1: 5 DCLKs, Phase 2: 7 DCLKs
                0x72,
            },
            1
        ),
        TAG, "io tx param SSD1322_SET_PHASE_LENGTH failed"
    );

    // Commenting this out doesn't seem to make any visible difference.
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_PRECHARGE_VOLTAGE,
            (uint8_t[]){
                // Set to max, 0.60 x Vcc
                0x1F,
            },
            1
        ),
        TAG, "io tx param SSD1322_SET_PRECHARGE_VOLTAGE failed"
    );

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_DISPLAY_ENHANCE,
            (uint8_t[]){
                // Enable external VSL, Enhanced low GS display quality
                0xA0,
                0xFD,
            },
            2
        ),
        TAG, "io tx param SSD1322_DISPLAY_ENHANCE failed"
    );

    if (true) {
        // Pixels outside of the screen area are initialised to random noise and the controller wastes clock cycles
        // driving them. The controller scans the full horizontal width between rows 0 and the multiplex ratio. Use our
        // screen buffer to zero out all the GDDRAM blocks within this range, slightly reducing power draw.

        // Our screen buffer is smaller than the full GDDRAM area, so we need to chunk it up.
        // TODO: In theory, we only need the left and right, and only display height.
        uint8_t regions[] = {
            // left, right, top, bottom
            // 0x00, 0x77, 0x00, 0x7F, // Full GDDRAM area (30720 bytes)
            // 0x1C, 0x5B, 0x00, 0x3F, // Visible display area (8192 bytes)
            0x00, 0x1B, 0x00, 0x7F, // Left of the screen, full height (7168 bytes)
            0x5C, 0x77, 0x00, 0x7F, // Right of the screen, full height (7168 bytes)
            0x1C, 0x5B, 0x40, 0x7F, // Below the screen, screen width (8192 bytes)
        };

        for (size_t i = 0; i < sizeof(regions); i += 4) {
            uint8_t left = regions[i];
            uint8_t right = regions[i + 1];
            uint8_t top = regions[i + 2];
            uint8_t bottom = regions[i + 3];
            ESP_LOGD(TAG, "Clearing region [%d-%d, %d-%d]", left, right, top, bottom);

            size_t count = (((right - left) + 1) * 2) * ((bottom - top) + 1);
            assert(count <= SCREEN_BUFFER_SIZE);

            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_param(
                    io, SSD1322_SET_COLUMN_ADDRESS,
                    (uint8_t[]){
                        left,
                        right,
                    },
                    2
                ),
                TAG, "io tx param SSD1322_SET_COLUMN_ADDRESS failed"
            );

            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_param(
                    io, SSD1322_SET_ROW_ADDRESS,
                    (uint8_t[]){
                        top,
                        bottom,
                    },
                    2
                ),
                TAG, "io tx param SSD1322_SET_ROW_ADDRESS failed"
            );

            ESP_RETURN_ON_ERROR(
                esp_lcd_panel_io_tx_color(io, SSD1322_WRITE_RAM, ssd1322->screen_buffer, count), TAG,
                "io tx color failed"
            );
        }
    }

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_COLUMN_ADDRESS,
            (uint8_t[]){
                // The left and right screen extents.
                0x1C,
                0x5B,
            },
            2
        ),
        TAG, "io tx param SSD1322_SET_COLUMN_ADDRESS failed"
    );

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_ROW_ADDRESS,
            (uint8_t[]){
                // The top and bottom screen extents.
                0x00,
                0x3F,
            },
            2
        ),
        TAG, "io tx param SSD1322_SET_ROW_ADDRESS failed"
    );

    ESP_LOGD(TAG, "panel init done, waking from sleep");

    // After reset, the display is asleep in normal display mode with random data in the buffer.
    // Match built-in esp_lcd drivers by blanking the display and waking it from sleep.
    // The user will be able to set an image before calling esp_lcd_panel_disp_on_off, which will complete instantly,
    // unlike waking from sleep.
    ESP_RETURN_ON_ERROR(panel_ssd1322_disp_on_off(panel, false), TAG, "panel_ssd1322_disp_on_off failed");
    ESP_RETURN_ON_ERROR(panel_ssd1322_disp_sleep(panel, false), TAG, "panel_ssd1322_disp_sleep failed");

    return ESP_OK;
}

/**
 * In raw mode, color_data must be prepared as 8bpp where the high and low 4 bits of each pixel are the same.
 * For example, for a 2x2 bitmap of 50% grey (0x7), the buffer should be [0x77, 0x77, 0x77, 0x77]
 */
static esp_err_t panel_ssd1322_draw_bitmap(
    esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data
) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;
    
    ESP_LOGV(TAG, "draw_bitmap(%d, %d, %d, %d)", x_start, y_start, x_end, y_end);

    ESP_RETURN_ON_FALSE(x_start >= 0, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: x_start out of bounds");
    ESP_RETURN_ON_FALSE(y_start >= 0, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: y_start out of bounds");
    ESP_RETURN_ON_FALSE(x_end <= SSD1322_PANEL_WIDTH, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: x_end out of bounds");
    ESP_RETURN_ON_FALSE(y_end <= SSD1322_PANEL_HEIGHT, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: y_end out of bounds");
    ESP_RETURN_ON_FALSE(x_start < x_end, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: x_start >= x_end");
    ESP_RETURN_ON_FALSE(y_start < y_end, ESP_ERR_INVALID_ARG, TAG, "draw_bitmap: y_start >= y_end");

    // TODO: Implement a raw mode to opt out of this conversion and buffering.
    // Mirror the top 4 bits of each pixel to the bottom 4 bits to match the hardware alignment.
    for (int y = y_start; y < y_end; ++y) {
        for (int x = x_start; x < x_end; ++x) {
            uint8_t pixel = ((uint8_t *)color_data)[((y - y_start) * (x_end - x_start)) + (x - x_start)];
            ssd1322->screen_buffer[(y * SSD1322_PANEL_WIDTH) + x] = (pixel & 0xF0) | (pixel >> 4);
        }
    }

    // esp_lcd_panel_io_tx_color doesn't support a stride, so the best we can do is full rows at a time.
    // TODO: Remember the last set column and row addresses, so we don't have to send them every time.
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_ROW_ADDRESS,
            (uint8_t[]){
                // The top and bottom screen extents.
                y_start,
                y_end - 1,
            },
            2
        ),
        TAG, "io tx param SSD1322_SET_ROW_ADDRESS failed"
    );

    size_t offset = y_start * SSD1322_PANEL_WIDTH;
    size_t count = (y_end - y_start) * SSD1322_PANEL_WIDTH;

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_color(io, SSD1322_WRITE_RAM, &ssd1322->screen_buffer[offset], count), TAG,
        "io tx color failed"
    );

    return ESP_OK;
}

static esp_err_t panel_ssd1322_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    int command = 0;
    if (ssd1322->is_display_blanked) {
        if (invert_color_data) {
            command = SSD1322_ENTIRE_DISPLAY_ON;
        } else {
            command = SSD1322_ENTIRE_DISPLAY_OFF;
        }
    } else {
        if (invert_color_data) {
            command = SSD1322_INVERSE_DISPLAY;
        } else {
            command = SSD1322_NORMAL_DISPLAY;
        }
    }

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "io tx param SSD1322 display command 0x%02X failed",
        command
    );

    ssd1322->is_display_inverted = invert_color_data;

    return ESP_OK;
}

static esp_err_t panel_ssd1322_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    uint8_t remap_flags = SSD1322_REMAP_FLAG_NIBBLE_ORDER;

    if (mirror_x) {
        remap_flags |= SSD1322_REMAP_FLAG_COLUMN_ORDER;
    }

    if (mirror_y) {
        remap_flags |= SSD1322_REMAP_FLAG_ROW_ORDER;
    }

    ESP_LOGD(TAG, "panel_ssd1322_mirror(%d, %d) = %02X", mirror_x, mirror_y, remap_flags);

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(
            io, SSD1322_SET_REMAP,
            (uint8_t[]){
                remap_flags,
                // Dual COM mode
                0x11,
            },
            2
        ),
        TAG, "io tx param SSD1322_SET_REMAP failed"
    );

    return ESP_OK;
}

static esp_err_t panel_ssd1322_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    int command = 0;
    if (ssd1322->is_display_inverted) {
        if (on_off) {
            command = SSD1322_INVERSE_DISPLAY;
        } else {
            command = SSD1322_ENTIRE_DISPLAY_ON;
        }
    } else {
        if (on_off) {
            command = SSD1322_NORMAL_DISPLAY;
        } else {
            command = SSD1322_ENTIRE_DISPLAY_OFF;
        }
    }

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "io tx param SSD1322 display command 0x%02X failed",
        command
    );

    ssd1322->is_display_blanked = !on_off;

    return ESP_OK;
}

static esp_err_t panel_ssd1322_disp_sleep(esp_lcd_panel_t *panel, bool sleep) {
    ssd1322_panel_t *ssd1322 = __containerof(panel, ssd1322_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1322->io;

    int command = 0;
    if (sleep) {
        command = SSD1322_SLEEP_MODE_ON;
    } else {
        command = SSD1322_SLEEP_MODE_OFF;
    }

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "io tx param SSD1322_SLEEP_MODE_ON/OFF failed"
    );

    // There's a 200ms delay before the screen is actually ready
    vTaskDelay(pdMS_TO_TICKS(200));

    return ESP_OK;
}
