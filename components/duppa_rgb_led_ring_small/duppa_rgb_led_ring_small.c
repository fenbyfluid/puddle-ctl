#include "duppa_rgb_led_ring_small.h"

#include "esp_check.h"
#include "esp_log.h"

#define IS31FL3746A_REG_CONFIGURATION 0x50
#define IS31FL3746A_REG_GLOBAL_CURRENT 0x51
#define IS31FL3746A_REG_PULL_RESISTORS 0x52
#define IS31FL3746A_REG_OSDE_STATUS_BASE 0x53
#define IS31FL3746A_REG_TEMP_STATUS 0x5F
#define IS31FL3746A_REG_SPREAD_SPECTRUM 0x60
#define IS31FL3746A_REG_RESET 0x8F
#define IS31FL3746A_REG_PWM_FREQ_ENABLE 0xE0
#define IS31FL3746A_REG_PWM_FREQ_SETTING 0xE2

#define IS31FL3746A_REG_ID 0xFC
#define IS31FL3746A_REG_COMMAND 0xFD
#define IS31FL3746A_REG_COMMAND_UNLOCK 0xFE

#define IS31FL3746A_BANK_PWM 0x00
#define IS31FL3746A_BANK_CONFIG 0x01

static const char *TAG = "IS31FL3746A";

static const uint8_t LED_MAP[IS31FL3746A_LED_COUNT] = {
    0,  6, 12, 18,  1,  7, 13, 19,
    2,  8, 14, 20,  3,  9, 15, 21,
    4, 10, 16, 22,  5, 11, 17, 23,
};

#define LED_BUFFER_SIZE (IS31FL3746A_LED_COUNT * 3)

typedef struct {
    uint32_t xfer_timeout_ms;
    uint8_t address;
    i2c_master_dev_handle_t i2c_handle;
    uint8_t current_bank;
    uint8_t scale_buffer[LED_BUFFER_SIZE];
    uint8_t scale_range[2];
    uint8_t color_buffer[LED_BUFFER_SIZE];
    uint8_t color_range[2];
} is31fl3746a_dev_t;

esp_err_t is31fl3746a_create(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, uint32_t scl_speed_hz, is31fl3746a_handle_t *handle_ret) {
    ESP_LOGD(TAG, "Creating IS31FL3746A handle for device address 0x%02X", device_address);

    is31fl3746a_dev_t *device = calloc(1, sizeof(is31fl3746a_dev_t));
    ESP_RETURN_ON_FALSE(device, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    device->xfer_timeout_ms = 100;
    device->address = device_address;
    device->scale_range[0] = 0xFF;
    device->color_range[0] = 0xFF;

    esp_err_t ret = ESP_OK;

    i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };

    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &device->i2c_handle), err, TAG, "Failed to add new I2C device");
    assert(device->i2c_handle);

    assert(handle_ret);
    *handle_ret = device;

    return ret;

err:
    is31fl3746a_delete(device);

    return ret;
}

esp_err_t is31fl3746a_delete(is31fl3746a_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Deleting IS31FL3746A handle");

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    esp_err_t ret = ESP_OK;
    if (device->i2c_handle) {
        ret = i2c_master_bus_rm_device(device->i2c_handle);
    }

    free(device);

    return ret;
}

esp_err_t is31fl3746a_select_bank(is31fl3746a_handle_t handle, uint8_t bank) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;
    if (device->current_bank == bank) {
        return ESP_OK;
    }

    uint8_t command[2] = {IS31FL3746A_REG_COMMAND_UNLOCK, 0xC5};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 2, device->xfer_timeout_ms), TAG, "Failed to unlock command register");

    command[0] = IS31FL3746A_REG_COMMAND;
    command[1] = bank;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 2, device->xfer_timeout_ms), TAG, "Failed to switch bank");

    device->current_bank = bank;

    return ESP_OK;
}

esp_err_t is31fl3746a_reg_read(is31fl3746a_handle_t handle, uint8_t bank, uint8_t reg, uint8_t *value) {
    if (!handle || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    ESP_RETURN_ON_ERROR(is31fl3746a_select_bank(handle, bank), TAG, "Failed to select bank");

    uint8_t command = reg;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(device->i2c_handle, &command, 1, value, 1, device->xfer_timeout_ms), TAG, "Failed to read register");

    return ESP_OK;
}

esp_err_t is31fl3746a_reg_write(is31fl3746a_handle_t handle, uint8_t bank, uint8_t reg, uint8_t value) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    ESP_RETURN_ON_ERROR(is31fl3746a_select_bank(handle, bank), TAG, "Failed to select bank");

    uint8_t command[2] = {reg, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 2, device->xfer_timeout_ms), TAG, "Failed to write register");

    return ESP_OK;
}

esp_err_t is31fl3746a_init(is31fl3746a_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    uint8_t command = IS31FL3746A_REG_ID;
    uint8_t response;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(device->i2c_handle, &command, 1, &response, 1, device->xfer_timeout_ms), TAG, "Failed to read ID code");

    uint8_t expected_id = 0xA0 | ((device->address & 0xF) << 1);
    if (response != expected_id) {
        ESP_LOGE(TAG, "Unexpected ID code: 0x%02X, expected 0x%02X", response, expected_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(is31fl3746a_reg_write(handle, IS31FL3746A_BANK_CONFIG, IS31FL3746A_REG_RESET, 0xAE), TAG, "Failed to reset");

    // Recommended configuration
    // Spread spectrum = enabled, 15%, 820us
    ESP_RETURN_ON_ERROR(is31fl3746a_reg_write(handle, IS31FL3746A_BANK_CONFIG, IS31FL3746A_REG_SPREAD_SPECTRUM, 0b0010110), TAG, "Failed to set spread spectrum");
    
    // Enable normal operation mode
    ESP_RETURN_ON_ERROR(is31fl3746a_reg_write(handle, IS31FL3746A_BANK_CONFIG, IS31FL3746A_REG_CONFIGURATION, 0x01), TAG, "Failed to enable normal operation mode");

    return ESP_OK;
}

esp_err_t is31fl3746a_set_global_scale(is31fl3746a_handle_t handle, uint8_t scale) {
    return is31fl3746a_reg_write(handle, IS31FL3746A_BANK_CONFIG, IS31FL3746A_REG_GLOBAL_CURRENT, scale);
}

esp_err_t is31fl3746a_set_led_helper(uint8_t *buffer, uint8_t *range, uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (led_index >= IS31FL3746A_LED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Remap the LED index to the actual register address
    led_index = LED_MAP[led_index];

    bool changed = false;

    if (buffer[(led_index * 3) + 0] != b) {
        buffer[(led_index * 3) + 0] = b;
        changed = true;
    }

    if (buffer[(led_index * 3) + 1] != g) {
        buffer[(led_index * 3) + 1] = g;
        changed = true;
    }

    if (buffer[(led_index * 3) + 2] != r) {
        buffer[(led_index * 3) + 2] = r;
        changed = true;
    }

    if (!changed) {
         // No change, don't need to update the range
        return ESP_OK;
    }

    if (range[0] == 0xFF) {
        range[0] = led_index;
        range[1] = led_index;
    } else {
        if (led_index < range[0]) {
            range[0] = led_index;
        }
        if (led_index > range[1]) {
            range[1] = led_index;
        }
    }

    return ESP_OK;
}

esp_err_t is31fl3746a_flush_led_helper(is31fl3746a_handle_t handle, uint8_t bank, uint8_t *buffer, uint8_t *range) {
    // No values are dirty, do nothing
    if (range[0] == 0xFF) {
        return ESP_OK;
    }

    assert(range[0] < IS31FL3746A_LED_COUNT && range[1] < IS31FL3746A_LED_COUNT);
    assert(range[0] <= range[1]);

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    ESP_RETURN_ON_ERROR(is31fl3746a_select_bank(handle, bank), TAG, "Failed to select bank");

    // Start address for the first LED
    uint8_t reg = 0x01 + (range[0] * 3);
    size_t bytes = (((range[1] - range[0]) + 1) * 3); // Register + bytes of LED data to write
    assert(bytes <= LED_BUFFER_SIZE);

    i2c_master_transmit_multi_buffer_info_t command[2] = {
        {
            .write_buffer = &reg,
            .buffer_size = 1,
        },
        {
            .write_buffer = &buffer[reg - 1],
            .buffer_size = bytes,
        }
    };

    ESP_RETURN_ON_ERROR(i2c_master_multi_buffer_transmit(device->i2c_handle, command, 2, device->xfer_timeout_ms), TAG, "Failed to write");

    // Reset the dirty range
    range[0] = 0xFF;
    range[1] = 0x00;

    return ESP_OK;
}

esp_err_t is31fl3746a_set_led_scale(is31fl3746a_handle_t handle, uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    return is31fl3746a_set_led_helper(device->scale_buffer, device->scale_range, led_index, r, g, b);
}

esp_err_t is31fl3746a_flush_led_scale(is31fl3746a_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    return is31fl3746a_flush_led_helper(handle, IS31FL3746A_BANK_CONFIG, device->scale_buffer, device->scale_range);
}

esp_err_t is31fl3746a_set_led_color(is31fl3746a_handle_t handle, uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    return is31fl3746a_set_led_helper(device->color_buffer, device->color_range, led_index, r, g, b);
}

esp_err_t is31fl3746a_flush_led_color(is31fl3746a_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    is31fl3746a_dev_t *device = (is31fl3746a_dev_t *)handle;

    return is31fl3746a_flush_led_helper(handle, IS31FL3746A_BANK_PWM, device->color_buffer, device->color_range);
}