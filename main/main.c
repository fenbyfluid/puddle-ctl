/**
 * Must have:
 * - USB HID interface, send current values to host, receive current status
 * - Show current state on displays
 * - Provide reasonable feedback via LED rings
 * 
 * Should have:
 * - On-screen configuration UI and persistent storage
 * - Option to bypass USB init by holding a button at boot
 * 
 * Nice to have:
 * - Opt-in USB CDC console output for debugging
 * - USB boot-to-DFU support, with a minimal DFU firmware doing OTA update
 * - BT HID interface for wireless support
 * 
 * Maybe have:
 * - Wi-Fi client interface for wireless support
 * 
 * TODO notes:
 * - Consider burning USB_PHY_SEL eFuse to stop JTAG/Serial trying to enumerate during bootloader
 *   This'll limit our debugging options, but we'll still be able to use the boot button to enter bootloader DFU mode
 * 
 */

#include <inttypes.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl.h"

#include "esp_lcd_panel_ssd1322.h"
#include "duppa_i2c_encoder.h"
#include "duppa_rgb_led_ring_small.h"

LV_FONT_DECLARE(monogram)
lv_font_t const *default_font = &monogram;

#define I2C_CLOCK_SPEED 100000 // 100 kHz - need stronger pull-ups for 400 kHz
#define OLED_SPI_CLOCK_SPEED (8 * 1000 * 1000) // We should be able to go up to 10 MHz - according to Logic, 10 oscillates between 8 and 12
#define OLED_SPI_HOST SPI2_HOST

#define DISPLAY_COUNT 2
#define ENCODER_COUNT 4

static const char *TAG = "main";

const gpio_num_t VSENSOR_EN_GPIO = 7;
const gpio_num_t ONBOARD_LED_GPIO = 13;
const gpio_num_t I2C_SDA_GPIO = 3;
const gpio_num_t I2C_SCL_GPIO = 4;
const gpio_num_t ENCODER_INTR_GPIO = 5;
const gpio_num_t SPI_SCLK_GPIO = 36;
const gpio_num_t SPI_MOSI_GPIO = 35;
const gpio_num_t OLED_DC_GPIO = 8;
const gpio_num_t OLED_LEFT_CS_GPIO = 18;
const gpio_num_t OLED_RIGHT_CS_GPIO = 17;
const gpio_num_t OLED_RESET_GPIO = 14;

lv_display_t *display_handles[DISPLAY_COUNT] = {NULL};
i2c_encoder_handle_t encoder_handles[ENCODER_COUNT] = {NULL};
is31fl3746a_handle_t ring_handles[ENCODER_COUNT] = {NULL};

void init_i2c_bus(void) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        // Our board has 10k pull-up resistors so we don't strictly need this, but it doesn't hurt and avoids a log warning.
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // I2C bus scan
    // for (uint8_t address = 0x08; address < 0x78; ++address) {
    //     esp_err_t ret = i2c_master_probe(bus_handle, address, 100);
    //     ESP_LOGI(TAG, "0x%02X = %s", address, esp_err_to_name(ret));
    // }

    // Current devices:
    //   0x36 = MAX17048 Battery Monitor
    //   0x43 OR 0x44 - 0x47 = I2CEncoder
    //   0x63 OR 0x64 - 0x67 = LED Ring Small

    // Try this to unblock any stuck devices first.
    i2c_master_bus_reset(bus_handle);

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        ESP_ERROR_CHECK(i2c_encoder_create(bus_handle, 0x44 + i, I2C_CLOCK_SPEED, &encoder_handles[i]));
        ESP_ERROR_CHECK(i2c_encoder_init(encoder_handles[i], true, STATUS_FLAG_BUTTON_UP | STATUS_FLAG_BUTTON_DOWN));
        ESP_ERROR_CHECK(i2c_encoder_set_range(encoder_handles[i], INT8_MIN, INT8_MAX, 1));
    }

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        ESP_ERROR_CHECK(is31fl3746a_create(bus_handle, 0x64 + i, I2C_CLOCK_SPEED, &ring_handles[i]));
        ESP_ERROR_CHECK(is31fl3746a_init(ring_handles[i]));
    }

    ESP_LOGI(TAG, "I2C bus and devices initialized");
}

static uint32_t lvgl_tick_callback(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void lvgl_flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    esp_lcd_panel_draw_bitmap(
        panel_handle,
        area->x1, area->y1,
        area->x2 + 1, area->y2 + 1,
        px_map);

    // Our draw bitmap implementation buffers internally, so we don't need LVGL to double-buffer.
    lv_display_flush_ready(disp);
}

void display_task_main(void *pvParameters) {
    spi_bus_config_t buscfg = {
        .sclk_io_num = SPI_SCLK_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SSD1322_PANEL_WIDTH * SSD1322_PANEL_HEIGHT,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    lv_init();
    lv_tick_set_cb(lvgl_tick_callback);

    ESP_LOGI(TAG, "LVGL initialized");

    for (int i = 0; i < DISPLAY_COUNT; ++i) {
        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = OLED_DC_GPIO,
            .cs_gpio_num = (i == 0) ? OLED_LEFT_CS_GPIO : OLED_RIGHT_CS_GPIO,
            .pclk_hz = OLED_SPI_CLOCK_SPEED,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 3,
            .trans_queue_depth = 10,
            .cs_ena_pretrans = 1,
            .cs_ena_posttrans = 1,
        };

        esp_lcd_panel_io_handle_t io_handle;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)OLED_SPI_HOST, &io_config, &io_handle));

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = (i == 0) ? OLED_RESET_GPIO : -1,
            .bits_per_pixel = 8,
        };

        esp_lcd_panel_handle_t panel_handle;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1322(io_handle, &panel_config, &panel_handle));

        if (i == 0) {
            // Wait for power to stabilize
            // We need to wait at least 1ms before reset, but 300ms before init
            // Reset itself waits 200ms, so we wait an extra 100ms to make up the difference
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

        display_handles[i] = lv_display_create(SSD1322_PANEL_WIDTH, SSD1322_PANEL_HEIGHT);

        // Using the PARTIAL render mode get's our flush callback the correct update coordinates.
        size_t draw_buffer_sz = SSD1322_PANEL_WIDTH * SSD1322_PANEL_HEIGHT;
        void *draw_buffer = calloc(1, draw_buffer_sz);
        lv_display_set_buffers(display_handles[i], draw_buffer, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

        lv_display_set_color_format(display_handles[i], LV_COLOR_FORMAT_L8);
        lv_display_set_user_data(display_handles[i], panel_handle);
        lv_display_set_flush_cb(display_handles[i], lvgl_flush_callback);

        // Use the draw buffer to clear the screen before turning it on.
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, SSD1322_PANEL_WIDTH, SSD1322_PANEL_HEIGHT, draw_buffer));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    }

    lv_theme_t *theme = lv_theme_mono_init(display_handles[0], true, default_font);
    lv_display_set_theme(display_handles[0], theme);
    lv_display_set_theme(display_handles[1], theme);

    // Notify main task that initialization is complete
    xTaskNotify((TaskHandle_t)pvParameters, 0, eNoAction);

    ESP_LOGI(TAG, "Display initialization complete, entering LVGL timer loop");

    for (;;) {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }
}

static void lvgl_read_encoder(lv_indev_t *indev, lv_indev_data_t *data) {
    i2c_encoder_handle_t encoder = lv_indev_get_user_data(indev);

    int8_t value = 0;
    ESP_ERROR_CHECK(i2c_encoder_read_value(encoder, &value));

    data->enc_diff = value;

    uint32_t flags = 0;
    ESP_ERROR_CHECK(i2c_encoder_poll_status(encoder, &flags));

    data->state = lv_indev_get_state(indev);

    if ((flags & STATUS_FLAG_BUTTON_UP) == STATUS_FLAG_BUTTON_UP) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else if ((flags & STATUS_FLAG_BUTTON_DOWN) == STATUS_FLAG_BUTTON_DOWN) {
        data->state = LV_INDEV_STATE_PRESSED;
    }
}

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    
    // SPI logging is very noisy
    esp_log_level_set("spi_master", ESP_LOG_INFO);

    ESP_LOGI(TAG, "Hello world!");

    // Enable the on-board LED
    ESP_ERROR_CHECK(gpio_reset_pin(ONBOARD_LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(ONBOARD_LED_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(ONBOARD_LED_GPIO, 1));

    // The VSENSOR power regulator is the high side of the on-board pull-ups for I2C
    // If we don't enable it, the I2C lines will try and pull current through the regulator
    ESP_ERROR_CHECK(gpio_reset_pin(VSENSOR_EN_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(VSENSOR_EN_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(VSENSOR_EN_GPIO, 1));

    // TODO: If the last reset was due to a panic, we may not want to continue automatically.
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %d", reset_reason);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Start initialization of the OLED displays and LVGL in a separate task
    xTaskCreatePinnedToCore(display_task_main, "OLED", 8192, xTaskGetCurrentTaskHandle(), 1, NULL, xPortGetCoreID());

    // Initialize I2C bus and devices in parallel with display init
    init_i2c_bus();

    // Wait for display initialization to complete
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    lv_obj_t *screen = lv_display_get_screen_active(display_handles[0]);

    lv_obj_t *roller = lv_roller_create(screen);
    lv_roller_set_options(roller,
        "Stroke Start\n"
        "Stroke Length\n"
        "Forward Velocity\n"
        "Forward Acceleration\n"
        "Forward Deceleration\n"
        "Backward Velocity\n"
        "Backward Acceleration\n"
        "Backward Deceleration", 
        LV_ROLLER_MODE_NORMAL);
    lv_obj_set_size(roller, lv_pct(100), lv_pct(100));
    lv_obj_set_style_border_width(roller, 0, 0);
    lv_obj_set_style_text_line_space(roller, 2, 0);
    lv_obj_set_style_anim_duration(roller, 0, 0);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_user_data(indev, encoder_handles[0]);
    lv_indev_set_read_cb(indev, lvgl_read_encoder);

    lv_group_t *group = lv_group_create();
    lv_group_add_obj(group, roller);
    lv_indev_set_group(indev, group);

    // TODO: Wait for an event from other tasks indicating initialization is complete
    ESP_LOGI(TAG, "Initialization complete");

    // xTaskCreatePinnedToCore(i2c_task_main, "I2C", 8192, NULL, 1, NULL, xPortGetCoreID());
    // xTaskCreatePinnedToCore(usb_task_main, "USB", 8192, NULL, 1, NULL, xPortGetCoreID());

    // Print heap memory usage statistics every 10 seconds
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Output heap memory usage statistics (total, free, percentage used, etc.)
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
        size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
        float used_percent = 100.0f * (total_heap - free_heap) / total_heap;

        ESP_LOGI(TAG, "Heap stats: Total = %u, Free = %u, Min Free = %u, Used = %.2f%%",
            total_heap, free_heap, min_free_heap, used_percent);
    }
}