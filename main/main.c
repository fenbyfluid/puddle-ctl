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
#include "esp_private/usb_phy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl.h"

#include "esp_lcd_panel_ssd1322.h"
#include "duppa_i2c_encoder.h"
#include "duppa_rgb_led_ring_small.h"

#include "usb.h"

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

const int32_t LEFT_SCREEN_LEFT_ENCODER_OFFSET = 17;
const int32_t LEFT_SCREEN_RIGHT_ENCODER_OFFSET = 107;
// const int32_t RIGHT_SCREEN_LEFT_ENCODER_OFFSET = 21;
// const int32_t RIGHT_SCREEN_RIGHT_ENCODER_OFFSET = 110;

SemaphoreHandle_t lvgl_mutex;

lv_display_t *display_handles[DISPLAY_COUNT] = {NULL};
lv_indev_t *encoder_indevs[ENCODER_COUNT] = {NULL};
i2c_encoder_handle_t encoder_handles[ENCODER_COUNT] = {NULL};
is31fl3746a_handle_t ring_handles[ENCODER_COUNT] = {NULL};

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

    // Fire off our own rotary event
    // This isn't quite valid, as the indev-sent ROTARY event param is supposed to be the target object,
    // and then the object-sent ROTARY event param is the rotation value, but we want to catch this globally.
    // We could use a custom event code instead, which is probably more correct, and would avoid confusion.
    if (value != 0) {
        int32_t rotation = value;
        lv_indev_send_event(indev, LV_EVENT_ROTARY, &rotation);
    }
}

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

    bool contrast_dimmed = false;

    for (;;) {
        uint32_t inactive_time = lv_display_get_inactive_time(NULL);
        if (inactive_time > 10000) {
            if (!contrast_dimmed) {
                ESP_LOGI(TAG, "Dimming display contrast due to inactivity");
                panel_ssd1322_set_contrast(lv_display_get_user_data(display_handles[0]), 0x10);
                panel_ssd1322_set_contrast(lv_display_get_user_data(display_handles[1]), 0x10);
                contrast_dimmed = true;
            }
        } else {
            if (contrast_dimmed) {
                ESP_LOGI(TAG, "Restoring display contrast due to activity");
                panel_ssd1322_set_contrast(lv_display_get_user_data(display_handles[0]), 0x9F);
                panel_ssd1322_set_contrast(lv_display_get_user_data(display_handles[1]), 0x9F);
                contrast_dimmed = false;
            }
        }

        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

        uint32_t time_till_next = lv_timer_handler();

        xSemaphoreGive(lvgl_mutex);

        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }
}

void handle_rotary_event(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);

    if (code != LV_EVENT_ROTARY) {
        ESP_LOGW(TAG, "Ignoring unexpected event code %s (%d)", lv_event_code_get_name(code), code);
        return;
    }

    int32_t rotation = lv_event_get_rotary_diff(event);

    lv_obj_t *line = lv_event_get_user_data(event);

    int32_t x = lv_obj_get_x(line);

    x += rotation * abs(rotation);

    lv_obj_set_x(line, x);
}

void handle_restart_button(lv_event_t *event) {
    lv_event_code_t code = lv_event_get_code(event);

    if (code != LV_EVENT_CLICKED) {
        ESP_LOGW(TAG, "Ignoring unexpected event code %s (%d)", lv_event_code_get_name(code), code);
        return;
    }

    ESP_LOGI(TAG, "Restart button clicked, restarting system");
    esp_restart();
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

    lv_init();
    lv_tick_set_cb(lvgl_tick_callback);
    ESP_LOGI(TAG, "LVGL initialized");

    lvgl_mutex = xSemaphoreCreateMutex();

    // Start initialization of the OLED displays and LVGL in a separate task
    xTaskCreatePinnedToCore(display_task_main, "OLED", 8192, xTaskGetCurrentTaskHandle(), 1, NULL, xPortGetCoreID());

    // Initialize I2C bus and devices in parallel with display init
    init_i2c_bus();

    // Wait for display initialization to complete
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    // Create an input device for each encoder
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        encoder_indevs[i] = lv_indev_create();
        lv_indev_set_type(encoder_indevs[i], LV_INDEV_TYPE_ENCODER);
        lv_indev_set_user_data(encoder_indevs[i], encoder_handles[i]);
        lv_indev_set_read_cb(encoder_indevs[i], lvgl_read_encoder);
    }

    ESP_LOGI(TAG, "Hardware initialization complete");

    {
        lv_indev_read(encoder_indevs[0]);
        lv_indev_state_t state = lv_indev_get_state(encoder_indevs[0]);

        if (state == LV_INDEV_STATE_RELEASED) {
            xTaskCreatePinnedToCore(usb_task_main, "USB", 8192, NULL, 1, NULL, xPortGetCoreID());
        } else if (state == LV_INDEV_STATE_PRESSED) {
            ESP_LOGI(TAG, "Skipping USB initialization due to button held at boot");

            // Switch the USB PHY back to Serial/Jtag mode, disabling OTG support.
            usb_phy_handle_t phy_hdl;
            usb_phy_config_t phy_conf = {
                .controller = USB_PHY_CTRL_SERIAL_JTAG,
            };
            ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &phy_hdl));
        }
    }

    // TODO: Create an initial test UI
    if (true) {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

        lv_obj_t *screen_left = lv_display_get_screen_active(display_handles[0]);
        lv_obj_t *label_left = lv_label_create(screen_left);
        lv_label_set_text(label_left, "Puddle");
        lv_obj_center(label_left);

        lv_obj_t *screen_right = lv_display_get_screen_active(display_handles[1]);
        lv_obj_t *label_right = lv_label_create(screen_right);
        lv_label_set_text(label_right, "Maker");
        lv_obj_center(label_right);

        xSemaphoreGive(lvgl_mutex);

        vTaskDelay(pdMS_TO_TICKS(1000));

        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);

        lv_display_set_default(display_handles[0]);
        lv_obj_t *left_screen = lv_obj_create(NULL);

        lv_obj_t *left_label = lv_label_create(left_screen);
        lv_label_set_text(left_label, "Start");
        lv_obj_align(left_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

        {
            lv_obj_update_layout(left_label);
            int32_t center = lv_obj_get_x(left_label) + (lv_obj_get_width(left_label) / 2);
            int32_t target_center = LEFT_SCREEN_LEFT_ENCODER_OFFSET;
            if (center < target_center) {
                lv_obj_set_x(left_label, target_center - center);
            }
        }

        lv_obj_t *right_label = lv_label_create(left_screen);
        lv_label_set_text(right_label, "End");
        lv_obj_align(right_label, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

        {
            lv_obj_update_layout(right_label);
            int32_t center = lv_obj_get_x(right_label) + (lv_obj_get_width(right_label) / 2);
            int32_t target_center = LEFT_SCREEN_RIGHT_ENCODER_OFFSET;
            if (center > target_center) {
                lv_obj_set_x(right_label, target_center - center);
            }
        }

        lv_screen_load_anim(left_screen, LV_SCREEN_LOAD_ANIM_MOVE_TOP, 500, 0, true);

        lv_display_set_default(display_handles[1]);
        lv_obj_t *right_screen = lv_obj_create(NULL);

        lv_obj_t *line = lv_obj_create(right_screen);
        lv_obj_remove_style_all(line);
        lv_obj_set_size(line, 1, 8);
        lv_obj_align(line, LV_ALIGN_BOTTOM_LEFT, 20, 0);
        lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(line, lv_color_white(), 0);
        
        i2c_encoder_set_led_color(encoder_handles[0], 255, 0, 0);
        lv_indev_add_event_cb(encoder_indevs[0], handle_rotary_event, LV_EVENT_ROTARY, line);

        lv_obj_t *button = lv_btn_create(right_screen);
        lv_obj_add_event_cb(button, handle_restart_button, LV_EVENT_CLICKED, NULL);
        lv_obj_t *button_label = lv_label_create(button);
        lv_label_set_text(button_label, "Restart");
        lv_obj_center(button);
        lv_obj_align(button, LV_ALIGN_CENTER, 0, 0);

        lv_group_t *group = lv_group_create();
        lv_group_add_obj(group, button);
        lv_indev_set_display(encoder_indevs[3], display_handles[1]);
        lv_indev_set_group(encoder_indevs[3], group);
        lv_group_focus_obj(button);
        i2c_encoder_set_led_color(encoder_handles[3], 0, 0, 255);

        lv_screen_load_anim(right_screen, LV_SCREEN_LOAD_ANIM_MOVE_TOP, 500, 0, true);

        xSemaphoreGive(lvgl_mutex);
    }

    // TODO: Test HID report sending. Need to abstract this away into our own concept of StrokeParams
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!tud_hid_ready()) {
            continue;
        }

        uint8_t report[CFG_TUD_HID_EP_BUFSIZE - 1] = {0x00};
        report[0] = 0x00; // Report ID
        report[1] = 0x11; // Param 1
        report[2] = 0x22; // Param 2
        report[3] = 0x33; // Param 3
        report[4] = 0x44; // Param 4

        // TODO: HID reports are only arriving at our host program when they're the full size of the endpoint buffer.
        tud_hid_report(1, report, sizeof(report));

        // ESP_LOGD(TAG, "Sent HID report");
    }

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