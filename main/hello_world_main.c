#include <inttypes.h>
#include <stdio.h>
#include <stdatomic.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_vfs.h"

#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "vfs_tinyusb.h"
#include "lvgl.h"

#include "esp_lcd_panel_ssd1322.h"
#include "duppa_i2c_encoder.h"
#include "duppa_rgb_led_ring_small.h"

#define I2C_CLOCK_SPEED 100000 // 100 kHz - need stronger pull-ups for 400 kHz
#define OLED_SPI_CLOCK_SPEED (8 * 1000 * 1000) // We should be able to go up to 10 MHz - according to Logic, 10 oscillates between 8 and 12
#define OLED_SPI_HOST SPI2_HOST

#define MAX_DISPLAY_COUNT 2
#define MAX_ENCODER_COUNT 4

const gpio_num_t STEMMA_QT_PWR_GPIO = 7;
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

static const char *TAG = "example";

static void init_stemma_power(void) {
    gpio_reset_pin(STEMMA_QT_PWR_GPIO);
    gpio_set_direction(STEMMA_QT_PWR_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(ONBOARD_LED_GPIO);
    gpio_set_direction(ONBOARD_LED_GPIO, GPIO_MODE_OUTPUT);
}

static void enable_stemma_power(bool enable) {
    gpio_set_level(STEMMA_QT_PWR_GPIO, enable);

    // Set the LED to match the power state
    gpio_set_level(ONBOARD_LED_GPIO, enable);

    // Wait some time for power to stabilize (probably not strictly needed)
    vTaskDelay(pdMS_TO_TICKS(100));
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

atomic_int_fast8_t encoder_values[MAX_ENCODER_COUNT] = {0};

void update_display_timer(lv_timer_t *timer) {
    lv_obj_t **labels = (lv_obj_t **)lv_timer_get_user_data(timer);

    for (int i = 0; i < MAX_ENCODER_COUNT; i++) {
        lv_label_set_text_fmt(labels[i], "%d", encoder_values[i]);
    }

    lv_timer_reset(timer);
}

lv_obj_t *create_label_with_container(lv_obj_t *parent, lv_align_t alignment) {
    lv_obj_t *container = lv_obj_create(parent);
    lv_obj_set_size(container, lv_pct(50), lv_pct(100));
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_align(container, alignment, 0, 0);

    lv_obj_t *label = lv_label_create(container);
    lv_label_set_text(label, "");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    return label;
}

void display_task_main(void *pvParameters) {
    esp_lcd_panel_io_handle_t io_handle[MAX_DISPLAY_COUNT] = {NULL, NULL};
    esp_lcd_panel_handle_t panel_handle[MAX_DISPLAY_COUNT] = {NULL, NULL};

    spi_bus_config_t buscfg = {
        .sclk_io_num = SPI_SCLK_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SSD1322_PANEL_WIDTH * SSD1322_PANEL_HEIGHT,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = OLED_DC_GPIO,
        .cs_gpio_num = OLED_LEFT_CS_GPIO,
        .pclk_hz = OLED_SPI_CLOCK_SPEED,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 3,
        .trans_queue_depth = 10,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)OLED_SPI_HOST, &io_config, &io_handle[0]));

    io_config.cs_gpio_num = OLED_RIGHT_CS_GPIO;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)OLED_SPI_HOST, &io_config, &io_handle[1]));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = OLED_RESET_GPIO,
        .bits_per_pixel = 8,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1322(io_handle[0], &panel_config, &panel_handle[0]));

    // The reset lines are joined together, so resetting one resets them both.
    panel_config.reset_gpio_num = -1;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1322(io_handle[1], &panel_config, &panel_handle[1]));

    // Wait for power to stabilize
    // We need to wait at least 1ms before reset, but 300ms before init
    // Reset itself waits 200ms, so we wait an extra 100ms to make up the difference
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; i < MAX_DISPLAY_COUNT; i++) {
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle[i]));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle[i]));
    }

    lv_obj_t *screen[MAX_DISPLAY_COUNT] = {NULL, NULL};

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    lv_tick_set_cb(lvgl_tick_callback);

    for (int i = 0; i < MAX_DISPLAY_COUNT; i++) {
        lv_display_t *display = lv_display_create(SSD1322_PANEL_WIDTH, SSD1322_PANEL_HEIGHT);

        size_t draw_buffer_sz = SSD1322_PANEL_WIDTH * SSD1322_PANEL_HEIGHT;
        void *draw_buffer = calloc(1, draw_buffer_sz);
        lv_display_set_buffers(display, draw_buffer, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

        lv_display_set_color_format(display, LV_COLOR_FORMAT_L8);
        lv_display_set_user_data(display, panel_handle[i]);
        lv_display_set_flush_cb(display, lvgl_flush_callback);

        // Use the draw buffer to clear the screen before turning it on.
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle[i], 0, 0, SSD1322_PANEL_WIDTH, SSD1322_PANEL_HEIGHT, draw_buffer));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle[i], true));

        screen[i] = lv_display_get_screen_active(display);

        // White text on a black background
        lv_obj_set_style_bg_color(screen[i], lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_text_color(screen[i], lv_color_hex(0xffffff), LV_PART_MAIN);
    }

    lv_obj_t *labels[MAX_ENCODER_COUNT] = {NULL};

    labels[0] = create_label_with_container(screen[0], LV_ALIGN_LEFT_MID);
    labels[1] = create_label_with_container(screen[0], LV_ALIGN_RIGHT_MID);
    labels[2] = create_label_with_container(screen[1], LV_ALIGN_LEFT_MID);
    labels[3] = create_label_with_container(screen[1], LV_ALIGN_RIGHT_MID);

    lv_timer_create(update_display_timer, 10, labels);

    // TODO: Implement an exit condition for this task.
    for (;;) {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }

    for (int i = 0; i < MAX_DISPLAY_COUNT; i++) {
        ESP_ERROR_CHECK(esp_lcd_panel_disp_sleep(panel_handle[i], true));
    }
    
    // TODO: De-init LVGL and remove the devices from the SPI bus

    vTaskDelete(NULL);
}

void i2c_task_main(void *pvParameters) {
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

    // Probe to determine if we have a dev board (single encoder) or a prod unit (quad encoders).
    int first_encoder = 0x00;
    int encoder_count = 0;
    if (i2c_master_probe(bus_handle, 0x44, 100) == ESP_OK) {
        first_encoder = 0x44;
        encoder_count = MAX_ENCODER_COUNT;
    } else if (i2c_master_probe(bus_handle, 0x43, 100) == ESP_OK) {
        first_encoder = 0x43;
        encoder_count = 1;
    }

    ESP_LOGI(TAG, "Detected first encoder at 0x%02X, count: %d", first_encoder, encoder_count);

    i2c_encoder_handle_t encoder_handles[MAX_ENCODER_COUNT] = {NULL};
    for (int i = 0; i < encoder_count; ++i) {
        ESP_ERROR_CHECK(i2c_encoder_create(bus_handle, first_encoder + i, I2C_CLOCK_SPEED, &encoder_handles[i]));
        ESP_ERROR_CHECK(i2c_encoder_init(encoder_handles[i], false, 0));
        ESP_ERROR_CHECK(i2c_encoder_set_range(encoder_handles[i], 0, IS31FL3746A_LED_COUNT, 1));
    }

    // ESP_ERROR_CHECK(i2c_encoder_setup_interrupt(ENCODER_INTR_GPIO, ...));

    is31fl3746a_handle_t ring_handles[MAX_ENCODER_COUNT] = {NULL};
    for (int i = 0; i < encoder_count; ++i) {
        ESP_ERROR_CHECK(is31fl3746a_create(bus_handle, 0x60 | (first_encoder + i), I2C_CLOCK_SPEED, &ring_handles[i]));
        ESP_ERROR_CHECK(is31fl3746a_init(ring_handles[i]));
        ESP_ERROR_CHECK(is31fl3746a_set_global_scale(ring_handles[i], 0x01));
        for (uint8_t j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
            ESP_ERROR_CHECK(is31fl3746a_set_led_scale(ring_handles[i], j, 0xFF, 0x7F, 0x7F));
        }
        ESP_ERROR_CHECK(is31fl3746a_flush_led_scale(ring_handles[i]));
    }

    for (int i = 0; i < encoder_count; ++i) {
        uint8_t led_scale = 8;
        ESP_ERROR_CHECK(i2c_encoder_set_led_color(encoder_handles[i], 0x7F / led_scale, 0xFF / led_scale, 0x7F / led_scale));
    }

    bool hid_report_changed = false;
    uint8_t hid_report[CFG_TUD_HID_EP_BUFSIZE - 1] = {0};
    
    // TODO: Implement an exit condition for this task.
    for (;;) {
        for (int i = 0; i < encoder_count; ++i) {
            // ESP_ERROR_CHECK(i2c_encoder_poll_status(encoder_handles[i], ...));

            int8_t value;
            ESP_ERROR_CHECK(i2c_encoder_read_value(encoder_handles[i], &value));

            encoder_values[i] = value;
            if (value != hid_report[i]) {
                hid_report[i] = value;
                hid_report_changed = true;
            }

            for (uint8_t j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
                if (value > j) {
                    ESP_ERROR_CHECK(is31fl3746a_set_led_color(ring_handles[i], j, 0xFF, 0xFF, 0xFF));
                } else {
                    ESP_ERROR_CHECK(is31fl3746a_set_led_color(ring_handles[i], j, 0x00, 0x00, 0x00));
                }
            }

            ESP_ERROR_CHECK(is31fl3746a_flush_led_color(ring_handles[i]));
        }

        if (hid_report_changed && tud_hid_ready()) {
            tud_hid_report(1, hid_report, sizeof(hid_report));
            hid_report_changed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int i = 0; i < encoder_count; ++i) {
        for (uint8_t j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
            ESP_ERROR_CHECK(is31fl3746a_set_led_color(ring_handles[i], j, 0x00, 0x00, 0x00));
        }

        ESP_ERROR_CHECK(is31fl3746a_flush_led_color(ring_handles[i]));

        ESP_ERROR_CHECK(i2c_encoder_set_led_color(encoder_handles[i], 0x00, 0x00, 0x00));

        is31fl3746a_delete(ring_handles[i]);
        i2c_encoder_delete(encoder_handles[i]);
    }

    i2c_del_master_bus(bus_handle);

    vTaskDelete(NULL);
}

static void device_event_handler(tinyusb_event_t *event, void *arg){
    ESP_LOGD(TAG, "USB event: %d", event->id);
}

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE - 1, HID_REPORT_ID(1)),
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (1 * TUD_HID_INOUT_DESC_LEN) + (1 * TUD_CDC_DESC_LEN))

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface number, string index, boot protocol, report descriptor len, EP data address (out, in), size, and polling interval
    TUD_HID_INOUT_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x01, 0x81, CFG_TUD_HID_EP_BUFSIZE, 10),

    // Interface number, string index, EP notification address and size, EP data address (out, in), and size.
    TUD_CDC_DESCRIPTOR(1, 5, 0x82, 8, 0x03, 0x83, CFG_TUD_ENDPOINT0_SIZE),
};

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    ESP_LOGD(TAG, "tud_hid_descriptor_report_cb(%d)", instance);

    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    ESP_LOGD(TAG, "tud_hid_get_report_cb(%d, %d, %d, %p, %d)", instance, report_id, report_type, buffer, reqlen);
    
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    if (report_id == 1 && report_type == HID_REPORT_TYPE_INPUT && reqlen >= 4) {
        buffer[0] = encoder_values[0];
        buffer[1] = encoder_values[1];
        buffer[2] = encoder_values[2];
        buffer[3] = encoder_values[3];

        return 4;
    }

    // report_id is automatically prefixed if non-zero.
    // TinyUSB will assert if the total length of the response is zero (i.e. we return 0 bytes for report ID 0).
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
// 
// When confused why the first byte is missing: https://github.com/hathach/tinyusb/issues/2929
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    ESP_LOGD(TAG, "tud_hid_set_report_cb(%d, %d, %d, %p, %d)", instance, report_id, report_type, buffer, bufsize);

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, bufsize, ESP_LOG_DEBUG);

    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
}

// The rest of these are optional callbacks.

// Invoked when received SET_IDLE request. return false will stall the request
// - Idle Rate = 0 : only send report if there is changes, i.e. skip duplication
// - Idle Rate > 0 : skip duplication, but send at least 1 report every idle rate (in unit of 4 ms).
bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate) {
    ESP_LOGD(TAG, "tud_hid_set_idle_cb(%d, %d)", instance, idle_rate);

    (void) instance;
    (void) idle_rate;

    return true;
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len) {
    // ESP_LOGD(TAG, "tud_hid_report_complete_cb(%d, %p, %d)", instance, report, len);

    // ESP_LOG_BUFFER_HEX_LEVEL(TAG, report, len, ESP_LOG_DEBUG);

    (void) instance;
    (void) report;
    (void) len;
}

// Invoked when a transfer wasn't successful
void tud_hid_report_failed_cb(uint8_t instance, hid_report_type_t report_type, uint8_t const* report, uint16_t xferred_bytes) {
    ESP_LOGD(TAG, "tud_hid_report_failed_cb(%d, %d, %p, %d)", instance, report_type, report, xferred_bytes);

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, report, xferred_bytes, ESP_LOG_DEBUG);

    (void) instance;
    (void) report_type;
    (void) report;
    (void) xferred_bytes;
}

// Placeholder, will be replaced with chip ID
char serial_number[32] = CONFIG_TINYUSB_DESC_SERIAL_STRING;

const char *descriptor_strings[] = {
    (char[]){0x09, 0x04},                    // 0: is supported language is English (0x0409)
    CONFIG_TINYUSB_DESC_MANUFACTURER_STRING, // 1: Manufacturer
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 2: Product
    serial_number,                           // 3: Serials, should use chip ID
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 4: HID Interface
    CONFIG_TINYUSB_DESC_CDC_STRING,          // 5: CDC Interface
    NULL                                     // NULL: Must be last. Indicates end of array
};

void usb_task_cleanup() {
    freopen("/dev/console", "w", stdout);
    freopen("/dev/console", "w", stderr);

    esp_vfs_tusb_cdc_unregister(TINYUSB_CDC_ACM_0);

    tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);

    ESP_ERROR_CHECK(tinyusb_driver_uninstall());
}

#include "esp_private/usb_phy.h"
#include "soc/rtc_cntl_reg.h"
#include "hal/usb_serial_jtag_hal.h"
#include "esp32s3/rom/usb/usb_dc.h"
#include "esp32s3/rom/usb/usb_persist.h"
#include "esp32s3/rom/usb/chip_usb_dw_wrapper.h"

#include "soc/usb_serial_jtag_reg.h"

// TODO: This isn't the best experience, as the USB PID changes when we restart, which causes it to get detected as a different serial port.
//       Investigate implementing USB DFU mode and doing it as an OTA update instead.
//       We should get ourselves a pair of custom PIDs from Espressif, and do both in USB-OTG mode.
void restart_to_bootloader(void *pvParameters) {
    ESP_LOGW(TAG, "Preparing to restart to bootloader");

    // TODO: Make sure the USB task is stopped before we do this. This is a bit of a hack for now.
    // TODO: Make sure we're not in the middle of sending a HID report either.
    usb_task_cleanup();

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Switch the USB PHY back to Serial/Jtag mode, disabling OTG support.
    usb_phy_handle_t phy_hdl;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_SERIAL_JTAG,
    };
    ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &phy_hdl));

    // Tell the USB peripheral we're going to enter ROM DFU mode.
    usb_dc_prepare_persist();
    chip_usb_set_persist_flags(USBDC_PERSIST_ENA);
    // chip_usb_set_persist_flags(USBDC_BOOT_DFU);

    // Tell the bootloader to enter download mode.
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    
    ESP_LOGI(TAG, "Restarting...");

    // Reset the system
    esp_restart();
}

void cdc_line_state_changed(int itf, cdcacm_event_t *event) {
    if (event->type == CDC_EVENT_LINE_CODING_CHANGED) {
        ESP_LOGI(TAG, "CDC line coding changed: %lu %u %u %u",
             event->line_coding_changed_data.p_line_coding->bit_rate,
             event->line_coding_changed_data.p_line_coding->data_bits,
             event->line_coding_changed_data.p_line_coding->parity,
             event->line_coding_changed_data.p_line_coding->stop_bits);

        return;
    }

    if (event->type != CDC_EVENT_LINE_STATE_CHANGED) {
        return;
    }

    static bool prev_dtr = false;
    static bool prev_rts = false;

    bool dtr = event->line_state_changed_data.dtr;
    bool rts = event->line_state_changed_data.rts;

    ESP_LOGI(TAG, "CDC line state changed: DTR = %d, RTS = %d", dtr, rts);

    static bool is_restarting = false;
    if (dtr && !rts && !prev_dtr && prev_rts && !is_restarting) {
        is_restarting = true;
        xTaskCreatePinnedToCore(restart_to_bootloader, "restart_to_bootloader", 4096, NULL, 5, NULL, xPortGetCoreID());
    }

    prev_dtr = dtr;
    prev_rts = rts;
}

static ssize_t myfs_write(int fd, const void *data, size_t size) {
    static _lock_t myfs_write_lock;
    static FILE *console_stream = NULL;
    static FILE *tusb_cdc_stream = NULL;

    _lock_acquire(&myfs_write_lock);

    if (!console_stream) {
        console_stream = fopen("/dev/console", "w");
        assert(console_stream != NULL && "Failed to open console stream");

        fputs("Console stream initialized.\n", console_stream);
    }

    if (!tusb_cdc_stream) {
        tusb_cdc_stream = fopen("/dev/tusb_cdc", "w");
        assert(tusb_cdc_stream != NULL && "Failed to open USB CDC stream");

        fputs("USB CDC stream initialized.\n", tusb_cdc_stream);
    }

    // We've seen the USB CDC stream get write blocked, whereas the console stream appears to buffer.
    // To avoid losing data, we only write to the console stream bytes that were successfully written to the USB CDC stream.
    // This is a lot simpler than two-way synchronization.
    // We return the number of bytes written to the console stream, which is the minimum of the two streams.
    // If the console stream ever fell behind, we'd write duplicate data to USB CDC, and would need the more complex logic.

    size_t tusb_cdc_written = fwrite(data, 1, size, tusb_cdc_stream);

    size_t console_written = fwrite(data, 1, tusb_cdc_written, console_stream);

    _lock_release(&myfs_write_lock);

    return console_written;
}

static ssize_t myfs_open(const char *path, int flags, int mode) {
    return 0;
}

static int myfs_close(int fd) {
    return 0;
}

static const esp_vfs_fs_ops_t myfs = {
    .write = &myfs_write,
    .open = &myfs_open,
    .close = &myfs_close,
};

void usb_task_main(void *pvParameters) {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY));
    snprintf(serial_number, sizeof(serial_number), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(device_event_handler);

    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED

    tusb_cfg.descriptor.string = descriptor_strings;
    while (tusb_cfg.descriptor.string[++tusb_cfg.descriptor.string_count] != NULL);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = cdc_line_state_changed,
        .callback_line_coding_changed = cdc_line_state_changed,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

    // This replaces the default console(s), we want to mirror instead.
    // We want output going to UART0, JTAG, and USB CDC.
    // We do this with a custom VFS that writes to both the console and USB CDC streams.
    // ESP_ERROR_CHECK(esp_tusb_init_console(TINYUSB_CDC_ACM_0));

    ESP_ERROR_CHECK(esp_vfs_tusb_cdc_register(TINYUSB_CDC_ACM_0, NULL));

    ESP_ERROR_CHECK(esp_vfs_register_fs("/dev/myconsole", &myfs, ESP_VFS_FLAG_STATIC, NULL));

    freopen("/dev/myconsole", "w", stdout);
    setlinebuf(stdout);
    freopen("/dev/myconsole", "w", stderr);
    setlinebuf(stderr);

    ESP_LOGI(TAG, "Console mirrored to USB CDC");

    // TODO: Considering performance, do we actually want to be using USB-CDC for the logging?
    //       Minimal latency is quite critical, so we need to ensure this doesn't have any contention with the HID interface.
    //       It's just a little frustrating for debugging if we need a phyiscal UART connection to see the logs.

    // TODO: Implement an exit condition for this task.
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Send a dummy report to test even when we have no encoders.
        // uint8_t hid_report[CFG_TUD_HID_EP_BUFSIZE - 1] = {0xCA};
        // tud_hid_report(1, hid_report, sizeof(hid_report));
    }

    usb_task_cleanup();

    vTaskDelete(NULL);
}

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "Hello world!");

    // TODO: If the last reset was due to a panic, we may not want to continue automatically.
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %d", reset_reason);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    init_stemma_power();

    // TODO: After a CPU reset (e.g. due to an abort) the stemma power still seems to be enabled here (and the on-board LED too).
    //       Need to understand why that is and how to safely reset it - is it enough to bring it low or are the I2C/SPI peripherals also still configured?

    // for (int i = 10; i >= 0; i--) {
    //     ESP_LOGI(TAG, "Waiting %d seconds...", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // The on-board I2C pull-up resistors are tied to VSENSOR. Fortunately we can spare the current now.
    enable_stemma_power(true);

    xTaskCreatePinnedToCore(display_task_main, "OLED", 8192, NULL, 1, NULL, xPortGetCoreID());
    xTaskCreatePinnedToCore(i2c_task_main, "I2C", 8192, NULL, 1, NULL, xPortGetCoreID());
    xTaskCreatePinnedToCore(usb_task_main, "USB", 8192, NULL, 1, NULL, xPortGetCoreID());

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

    // ESP_LOGI(TAG, "Restarting now.");
    // fflush(stdout);
    // esp_restart();
}
