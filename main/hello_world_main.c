#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1322.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logo.h"
#include "sdkconfig.h"

#define BLINK_GPIO         13
#define BUTTON_GPIO        6
#define STEMMA_QT_PWR_GPIO 7

static const char *TAG = "example";

SemaphoreHandle_t buttonSemaphore = NULL;

static void IRAM_ATTR on_button_click(void *arg) {
    (void)arg;

    static TickType_t lastInterrupt = 0;
    TickType_t thisInterrupt = xTaskGetTickCountFromISR();
    if (thisInterrupt < (lastInterrupt + pdMS_TO_TICKS(500))) {
        return;
    }
    lastInterrupt = thisInterrupt;

    xSemaphoreGive(buttonSemaphore);
}

static void init_button(void) {
    buttonSemaphore = xSemaphoreCreateBinary();

    gpio_reset_pin(BUTTON_GPIO);

    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    gpio_config(&button_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, on_button_click, NULL);

    gpio_intr_enable(BUTTON_GPIO);
}

static void wait_for_button(void) {
    while (!xSemaphoreTake(buttonSemaphore, portMAX_DELAY)) {
        /* wait for the button ISR */
    }

    ESP_LOGI(TAG, "button pressed!");
}

static void init_stemma_power(void) {
    gpio_reset_pin(STEMMA_QT_PWR_GPIO);
    gpio_set_direction(STEMMA_QT_PWR_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void enable_stemma_power(bool enable) {
    gpio_set_level(STEMMA_QT_PWR_GPIO, enable);

    // Set the LED to match the power state
    gpio_set_level(BLINK_GPIO, enable);
}

SemaphoreHandle_t colorTransSemaphore = NULL;

bool color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    // We can't log from in here.
    // ESP_LOGI(TAG, "color trans done!");

    xSemaphoreGive(colorTransSemaphore);

    return true;
}

void app_main(void) {
    printf("Hello world!\n");

    init_button();

    init_stemma_power();

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf(
        "This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "", (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : ""
    );

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf(
        "%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external"
    );

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Waiting %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Display tests.
    esp_lcd_panel_io_handle_t io_handle_left = NULL;
    esp_lcd_panel_handle_t panel_handle_left = NULL;
    esp_lcd_panel_io_handle_t io_handle_right = NULL;
    esp_lcd_panel_handle_t panel_handle_right = NULL;

    {
        spi_bus_config_t buscfg = {
            .sclk_io_num = 36,
            .mosi_io_num = 35,
            .miso_io_num = -1,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 128 * 64, // TODO: Absolute guess - one screen's worth
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature

        esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = 9,
            .cs_gpio_num = 10,
            .pclk_hz = 10 * 1000 * 1000, // We should be able to go up to 10
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 3,
            .trans_queue_depth = 10,
        };

        // Attach the LCD to the SPI bus
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle_left));

        io_config.cs_gpio_num = 11;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle_right));

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = 12,
            .bits_per_pixel = 8,
        };

        // Create LCD panel handle for SSD1322, with the SPI IO device handle
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1322(io_handle_left, &panel_config, &panel_handle_left));

        // The reset lines are joined together, so resetting one resets them both.
        panel_config.reset_gpio_num = -1;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1322(io_handle_right, &panel_config, &panel_handle_right));
    }

    enable_stemma_power(true);

    {
        // Wait for power to stabilize
        // We need to wait at least 1ms before reset, but 300ms before init
        // Reset itself waits 200ms, so we wait an extra 100ms to make up the difference
        vTaskDelay(pdMS_TO_TICKS(100));

        // The reset for the right panel is a no-op, see above.
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_left));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_right));

        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_left));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_right));

        colorTransSemaphore = xSemaphoreCreateBinary();

        // TODO: Experiment...
        ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(
            io_handle_left,
            &(esp_lcd_panel_io_callbacks_t){
                .on_color_trans_done = color_trans_done_cb,
            },
            NULL
        ));

        ESP_LOGI(TAG, "drawing an image");

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_left, 0, 0, 128, 64, logo_image));
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_right, 0, 0, 128, 64, logo_image));

        while (!xSemaphoreTake(colorTransSemaphore, portMAX_DELAY)) {
            /* wait for the DMA transaction to complete */
        }

        ESP_LOGI(TAG, "color trans done!");

        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_left, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_right, true));
    }

    wait_for_button();
    esp_lcd_panel_invert_color(panel_handle_left, true);
    wait_for_button();
    esp_lcd_panel_invert_color(panel_handle_left, false);

    for (uint8_t i = 0; i < 4; ++i) {
        wait_for_button();

        esp_lcd_panel_mirror(panel_handle_left, i == 1 || i == 3, i == 2 || i == 3);

        // The screen needs to be redrawn after changing mirror_x
        uint8_t pixel = 0xFF;
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_left, 0, 0, 1, 1, &pixel));
    }

    for (uint8_t contrast = 0; contrast <= 0x9F; contrast += 32) {
        wait_for_button();

        ESP_LOGI(TAG, "contrast = %d", contrast);

        panel_ssd1322_set_contrast(panel_handle_left, contrast);
    }

    uint8_t *empty_buffer = calloc(1, 128 * 64);

    wait_for_button();

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_left, 0, 0, 128, 64, empty_buffer));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_right, 0, 0, 128, 64, empty_buffer));

    for (uint8_t width = 1; width <= 128; ++width) {
        wait_for_button();

        // Enable 64 random pixels
        for (uint8_t i = 0; i < 64;) {
            uint32_t rand = esp_random();
            uint32_t x = rand & 0x7F;
            uint32_t y = (rand >> 7) & 0x3F;
            uint32_t xy = (y * 128) + x;

            // ESP_LOGI(TAG, "[%d-%d] = [%d, %d] = %d", width, i, x, y, xy);

            if (empty_buffer[xy] > 0) {
                continue;
            }

            empty_buffer[xy] = 0xFF;
            i++;
        }

        ESP_LOGI(TAG, "width = %d", width);

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_left, 0, 0, 128, 64, empty_buffer));
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle_right, 0, 0, 128, 64, empty_buffer));
    }

    wait_for_button();

    // for (int i = 10; i >= 0; i--) {
    //     printf("Powering off in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    {
        ESP_ERROR_CHECK(esp_lcd_panel_disp_sleep(panel_handle_left, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_sleep(panel_handle_right, true));

        // Wait for sleep before powering down
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Don't do this ever for now, as we're getting reverse current flow that needs troubleshooting.
    // TODO: We need to turn off the SPI interface before turning off the power.
    // enable_stemma_power(false);

    vTaskSuspend(NULL);

    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}
