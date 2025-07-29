#include <inttypes.h>
#include <stdio.h>
#include <stdatomic.h>
#include <string.h>

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

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
    } else {
        ESP_LOGW(TAG, "No encoders detected.");

        i2c_del_master_bus(bus_handle);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Detected first encoder at 0x%02X, count: %d", first_encoder, encoder_count);

    i2c_encoder_handle_t encoder_handles[MAX_ENCODER_COUNT] = {NULL};
    for (int i = 0; i < encoder_count; ++i) {
        ESP_ERROR_CHECK(i2c_encoder_create(bus_handle, first_encoder + i, I2C_CLOCK_SPEED, &encoder_handles[i]));
        ESP_ERROR_CHECK(i2c_encoder_init(encoder_handles[i], false));
        ESP_ERROR_CHECK(i2c_encoder_set_range(encoder_handles[i], 0, IS31FL3746A_LED_COUNT, 1));
    }

    ESP_ERROR_CHECK(i2c_encoder_setup_interrupt(ENCODER_INTR_GPIO, true));

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

    for (;;) {
        for (int i = 0; i < encoder_count; ++i) {
            // ESP_ERROR_CHECK(i2c_encoder_poll_status(encoder_handles[i]));

            int8_t value;
            ESP_ERROR_CHECK(i2c_encoder_read_value(encoder_handles[i], &value));

            encoder_values[i] = value;

            for (uint8_t j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
                if (value > j) {
                    ESP_ERROR_CHECK(is31fl3746a_set_led_color(ring_handles[i], j, 0xFF, 0xFF, 0xFF));
                } else {
                    ESP_ERROR_CHECK(is31fl3746a_set_led_color(ring_handles[i], j, 0x00, 0x00, 0x00));
                }
            }

            ESP_ERROR_CHECK(is31fl3746a_flush_led_color(ring_handles[i]));
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

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    printf("Hello world!\n");

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    init_stemma_power();

    // TODO: After a CPU reset (e.g. due to an abort) the stemma power still seems to be enabled here (and the on-board LED too).
    //       Need to understand why that is and how to safely reset it - is it enough to bring it low or are the I2C/SPI peripherals also still configured?

    // for (int i = 10; i >= 0; i--) {
    //     printf("Waiting %d seconds...\n", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // The on-board I2C pull-up resistors are tied to VSENSOR. Fortunately we can spare the current now.
    enable_stemma_power(true);

    xTaskCreatePinnedToCore(display_task_main, "OLED", 8192, NULL, 10, NULL, xPortGetCoreID());
    xTaskCreatePinnedToCore(i2c_task_main, "I2C", 8192, NULL, 10, NULL, xPortGetCoreID());

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

    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}
