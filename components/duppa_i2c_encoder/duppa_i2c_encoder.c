#include "duppa_i2c_encoder.h"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/task.h"

static const char *TAG = "I2CEncoder";

static const uint8_t ID_CODE = 0x53;

typedef enum {
    REG_GCONF    = 0x00,
    REG_GP1CONF  = 0x01,
    REG_GP2CONF  = 0x02,
    REG_GP3CONF  = 0x03,
    REG_INTCONF  = 0x04,
    REG_ESTATUS  = 0x05,
    REG_I2STATUS = 0x06,
    REG_FSTATUS  = 0x07,
    REG_CVAL4    = 0x08,
    REG_CVAL3    = 0x09,
    REG_CVAL2    = 0x0A,
    REG_CVAL1    = 0x0B,
    REG_CMAX4    = 0x0C,
    REG_CMAX3    = 0x0D,
    REG_CMAX2    = 0x0E,
    REG_CMAX1    = 0x0F,
    REG_CMIN4    = 0x10,
    REG_CMIN3    = 0x11,
    REG_CMIN2    = 0x12,
    REG_CMIN1    = 0x13,
    REG_ISTEP4   = 0x14,
    REG_ISTEP3   = 0x15,
    REG_ISTEP2   = 0x16,
    REG_ISTEP1   = 0x17,
    REG_RLED     = 0x18,
    REG_GLED     = 0x19,
    REG_BLED     = 0x1A,
    REG_GP1REG   = 0x1B,
    REG_GP2REG   = 0x1C,
    REG_GP3REG   = 0x1D,
    REG_ANTBOUNC = 0x1E,
    REG_DPPERIOD = 0x1F,
    REG_FADERGB  = 0x20,
    REG_FADEGP   = 0x21,
    REG_GAMRLED  = 0x27,
    REG_GAMGLED  = 0x28,
    REG_GAMBLED  = 0x29,
    REG_GAMMAGP1 = 0x2A,
    REG_GAMMAGP2 = 0x2B,
    REG_GAMMAGP3 = 0x2C,
    REG_GCONF2   = 0x30,
    REG_IDCODE   = 0x70,
    REG_VERSION  = 0x71,
    REG_EEPROM   = 0x80,
} i2c_encoder_register_t;

typedef enum {
    GCONF_FLAG_DTYPE = (1 << 0), // Data type: 0 = int, 1 = float
    GCONF_FLAG_WRAPE = (1 << 1), // Wrap enable: 0 = disable, 1 = enable
    GCONF_FLAG_DIRE = (1 << 2), // Direction: 0 = right, 1 = left
    GCONF_FLAG_IPUD = (1 << 3), // Internal pull-up disable: 0 = enable, 1 = disable
    GCONF_FLAG_RMOD = (1 << 4), // Resolution mode: 0 = X1, 1 = X2
    GCONF_FLAG_ETYPE = (1 << 5), // Encoder type: 0 = standard, 1 = RGB
    GCONF_FLAG_EBANK = (1 << 6), // EEPROM bank: 0 = first bank, 1 = second bank
    GCONF_FLAG_RESET = (1 << 7), // Reset: 0 = no reset, 1 = reset
} i2c_encoder_gconf_flag_t;

typedef enum {
    GCONF2_FLAG_CKSRC = (1 << 0), // Clock stretching: 0 = disable, 1 = enable
    GCONF2_FLAG_RELMOD = (1 << 1), // Relative mode: 0 = absolute, 1 = relative
} i2c_encoder_gconf2_flag_t;

typedef enum {
    STATUS_FLAG_IPUSHR = (1 << 0), 
    STATUS_FLAG_IPUSHP = (1 << 1),
    STATUS_FLAG_IPUSHD = (1 << 2),
    STATUS_FLAG_IRINC = (1 << 3),
    STATUS_FLAG_IRDEC = (1 << 4),
    STATUS_FLAG_IRMAX = (1 << 5),
    STATUS_FLAG_IRMIN = (1 << 6),
    STATUS_FLAG_INT2 = (1 << 7),
} i2c_encoder_status_flag_t;

typedef enum {
    STATUS2_FLAG_GP1_POS = (1 << 0), 
    STATUS2_FLAG_GP1_NEG = (1 << 1),
    STATUS2_FLAG_GP2_POS = (1 << 2),
    STATUS2_FLAG_GP2_NEG = (1 << 3),
    STATUS2_FLAG_GP3_POS = (1 << 4),
    STATUS2_FLAG_GP3_NEG = (1 << 5),
    STATUS2_FLAG_FADE = (1 << 6),
} i2c_encoder_secondary_status_flag_t;

typedef enum {
    FADE_FLAG_RLED = (1 << 0), 
    FADE_FLAG_GLED = (1 << 1),
    FADE_FLAG_BLED = (1 << 2),
    FADE_FLAG_GP1 = (1 << 3),
    FADE_FLAG_GP2 = (1 << 4),
    FADE_FLAG_GP3 = (1 << 5),
} i2c_encoder_fade_status_flag_t;

typedef enum {
    GAMMA_OFF = 0b000,
    GAMMA_1_0 = 0b001,
    GAMMA_1_8 = 0b010,
    GAMMA_2_0 = 0b011,
    GAMMA_2_2 = 0b100,
    GAMMA_2_4 = 0b101,
    GAMMA_2_6 = 0b110,
    GAMMA_2_8 = 0b111,
} i2c_encoder_gamma_mode_t;

typedef struct {
    uint32_t xfer_timeout_ms;
    i2c_master_dev_handle_t i2c_handle;
} i2c_encoder_dev_t;

esp_err_t i2c_encoder_create(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, uint32_t scl_speed_hz, i2c_encoder_handle_t *handle_ret) {
    ESP_LOGD(TAG, "Creating I2C Encoder handle for device address 0x%02X", device_address);

    i2c_encoder_dev_t *device = calloc(1, sizeof(i2c_encoder_dev_t));
    ESP_RETURN_ON_FALSE(device, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    device->xfer_timeout_ms = 100;

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
    i2c_encoder_delete(device);

    return ret;
}

esp_err_t i2c_encoder_delete(i2c_encoder_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Deleting I2C Encoder handle");

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    esp_err_t ret = ESP_OK;
    if (device->i2c_handle) {
        ret = i2c_master_bus_rm_device(device->i2c_handle);
    }

    free(device);

    return ret;
}

esp_err_t i2c_encoder_reg_read(i2c_encoder_handle_t handle, uint8_t reg, uint8_t *value) {
    if (!handle || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    uint8_t command = reg;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(device->i2c_handle, &command, 1, value, 1, device->xfer_timeout_ms), TAG, "Failed to read register");

    return ESP_OK;
}

esp_err_t i2c_encoder_reg_write(i2c_encoder_handle_t handle, uint8_t reg, uint8_t value) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    uint8_t command[2] = {reg, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 2, device->xfer_timeout_ms), TAG, "Failed to write register");

    return ESP_OK;
}

esp_err_t i2c_encoder_reg_write_s32(i2c_encoder_handle_t handle, uint8_t reg, int32_t value) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    uint8_t command[5] = {reg, (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 5, device->xfer_timeout_ms), TAG, "Failed to write register");

    return ESP_OK;
}

esp_err_t i2c_encoder_init(i2c_encoder_handle_t handle, bool relative_mode) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t response;
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_read(handle, REG_IDCODE, &response), TAG, "Failed to read device ID");
    if (response != ID_CODE) {
        ESP_LOGE(TAG, "Unexpected ID code: 0x%02X, expected 0x%02X", response, ID_CODE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t flags = GCONF_FLAG_IPUD | GCONF_FLAG_ETYPE | GCONF_FLAG_RESET;

    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write(handle, REG_GCONF, flags), TAG, "Failed to reset encoder");
    vTaskDelay(pdMS_TO_TICKS(10)); // DS says "The RESET command takes 400us to be executed", but oddly 5ms isn't enough.

    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write(handle, REG_GCONF, flags & ~GCONF_FLAG_RESET), TAG, "Failed to configure GCONF");
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write(handle, REG_GCONF2, GCONF2_FLAG_CKSRC | (relative_mode ? GCONF2_FLAG_RELMOD : 0)), TAG, "Failed to configure GCONF2");
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write(handle, REG_ANTBOUNC, 5), TAG, "Failed to configure ANTBOUNC"); // Intervals of 0.192ms, so ~1ms

    return ESP_OK;
}

esp_err_t i2c_encoder_set_range(i2c_encoder_handle_t handle, int8_t min, int8_t max, int8_t step) {
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write_s32(handle, REG_CMIN4, min), TAG, "Failed to set CMIN");
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write_s32(handle, REG_CMAX4, max), TAG, "Failed to set CMAX");
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_write_s32(handle, REG_ISTEP4, step), TAG, "Failed to set ISTEP");

    return ESP_OK;
}

esp_err_t i2c_encoder_read_value(i2c_encoder_handle_t handle, int8_t *value) {
    return i2c_encoder_reg_read(handle, REG_CVAL1, (uint8_t *)value);
}

esp_err_t i2c_encoder_set_led_color(i2c_encoder_handle_t handle, uint8_t r, uint8_t g, uint8_t b) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    uint8_t command[4] = {REG_RLED, r, g, b};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, command, 4, device->xfer_timeout_ms), TAG, "Failed to write LED color");

    return ESP_OK;
}

esp_err_t i2c_encoder_poll_status(i2c_encoder_handle_t handle) { 
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_encoder_dev_t *device = (i2c_encoder_dev_t *)handle;

    uint8_t status;
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_read(handle, REG_ESTATUS, &status), TAG, "Failed to read encoder status");

    ESP_LOGD(TAG, "Encoder status: 0x%02X", status);

    if ((status & STATUS_FLAG_INT2) == 0) {
        return ESP_OK;
    }

    uint8_t secondary_status;
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_read(handle, REG_I2STATUS, &secondary_status), TAG, "Failed to read secondary status");

    ESP_LOGD(TAG, "Encoder secondary status: 0x%02X", secondary_status);

    if ((secondary_status & STATUS2_FLAG_FADE) == 0) {
        return ESP_OK;
    }

    uint8_t fade_status;
    ESP_RETURN_ON_ERROR(i2c_encoder_reg_read(handle, REG_FSTATUS, &fade_status), TAG, "Failed to read fade status");

    ESP_LOGD(TAG, "Encoder fade status: 0x%02X", fade_status);

    return ESP_OK;
}

static EventGroupHandle_t encoder_event_group = NULL;
#define ENCODER_INTERRUPT_BIT (1 << 0)

static void IRAM_ATTR on_encoder_interrupt(void *arg) {
    (void)arg;

    ESP_DRAM_LOGI(TAG, "i2c encoder interrupt!");

    if (!encoder_event_group) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(encoder_event_group, ENCODER_INTERRUPT_BIT, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t i2c_encoder_setup_interrupt(gpio_num_t intr_gpio, bool enable) {
    gpio_reset_pin(intr_gpio);

    if (!encoder_event_group) {
        encoder_event_group = xEventGroupCreate();
        if (!encoder_event_group) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (enable) {
        gpio_config_t intr_conf = {
            .pin_bit_mask = (1ULL << intr_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };

        gpio_config(&intr_conf);

        gpio_isr_handler_add(intr_gpio, on_encoder_interrupt, NULL);

        gpio_intr_enable(intr_gpio);
    } else {
        gpio_intr_disable(intr_gpio);

        gpio_isr_handler_remove(intr_gpio);
    }

    return ESP_OK;
}

esp_err_t i2c_encoder_wait_for_interrupt(TickType_t ticks_to_wait) { 
    if (!encoder_event_group) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        encoder_event_group,
        ENCODER_INTERRUPT_BIT,
        pdTRUE,
        pdFALSE,
        ticks_to_wait
    );

    return (bits & ENCODER_INTERRUPT_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}
