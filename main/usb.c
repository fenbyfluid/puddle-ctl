#include "usb.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_mac.h"

#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"

static const char *TAG = "USB";

// Placeholder, will be replaced with chip ID
char serial_number[32] = CONFIG_TINYUSB_DESC_SERIAL_STRING;

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_HID_INTERFACE,
    STRID_CDC_INTERFACE,
    STRID_DFU_RT_INTERFACE,
    STRID_TOTAL,
};

const char *descriptor_strings[] = {
    (char[]){0x09, 0x04},                    // 0: is supported language is English (0x0409)
    CONFIG_TINYUSB_DESC_MANUFACTURER_STRING, // 1: Manufacturer
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 2: Product
    serial_number,                           // 3: Serials, should use chip ID
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 4: HID Interface
    CONFIG_TINYUSB_DESC_CDC_STRING,          // 5: CDC Interface
    // CONFIG_TINYUSB_DESC_PRODUCT_STRING,      // 6: DFU Runtime Interface
    NULL,                                    // NULL: Must be last. Indicates end of array
};

// TODO: Having trouble getting interfaces to be detected when DFU runtime is included.
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (1 * TUD_HID_INOUT_DESC_LEN) + (1 * TUD_CDC_DESC_LEN) + (0 * TUD_DFU_RT_DESC_LEN))

enum {
    ITF_NUM_HID = 0,
    ITF_NUM_CDC,
    // ITF_NUM_DFU_RT,
    ITF_NUM_TOTAL,
};

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE - 1, HID_REPORT_ID(1)),
};

static const uint8_t configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, STRID_LANGID, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface number, string index, boot protocol, report descriptor len, EP data address (out, in), size, and polling interval
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, STRID_HID_INTERFACE, false, sizeof(hid_report_descriptor), 0x01, 0x81, CFG_TUD_HID_EP_BUFSIZE, 10),

    // Interface number, string index, EP notification address and size, EP data address (out, in), and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC_INTERFACE, 0x82, 8, 0x03, 0x83, CFG_TUD_ENDPOINT0_SIZE),

    // Interface number, string index, attributes, detach timeout, transfer size
    // TUD_DFU_RT_DESCRIPTOR(ITF_NUM_DFU_RT, STRID_DFU_RT_INTERFACE, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT| DFU_ATTR_WILL_DETACH, 1000, 4096),
};

// Mandatory HID callbacks follow

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

    // TODO: Is this still called? Is it possibly needed for other types of USB host interface?
    if (report_id == 1 && report_type == HID_REPORT_TYPE_INPUT && reqlen >= 4) {
        buffer[0] = 0;
        buffer[1] = 0;
        buffer[2] = 0;
        buffer[3] = 0;

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

// Optional HID callbacks follow

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

// Mandatory DFU callbacks follow

// Invoked when a DFU_DETACH request is received and bitWillDetach is set
void tud_dfu_runtime_reboot_to_dfu_cb(void) {
    ESP_LOGI(TAG, "DFU detach request received, rebooting to DFU mode");

    // TODO: Implement reboot to DFU mode
    //       Longer term we want to do this via a custom app firmware slot (for a custom PID),
    //       but the bootloader implementation will do for now.
    // esp_restart();
}

// End of the esp_tinyusb callbacks

static void device_event_handler(tinyusb_event_t *event, void *arg){
    ESP_LOGD(TAG, "USB event: %d", event->id);
}

void usb_task_main(void *pvParameters) {
    ESP_LOGI(TAG, "Starting USB task");

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY));
    snprintf(serial_number, sizeof(serial_number), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGD(TAG, "Got MAC address: %s", serial_number);

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(device_event_handler);

    tusb_cfg.descriptor.full_speed_config = configuration_descriptor;
#if (TUD_OPT_HIGH_SPEED)
    tusb_cfg.descriptor.high_speed_config = configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED

    tusb_cfg.descriptor.string = descriptor_strings;
    while (tusb_cfg.descriptor.string[++tusb_cfg.descriptor.string_count] != NULL);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

    // This replaces the default console(s), we want to mirror instead.
    // We want output going to UART0, JTAG, and USB CDC.
    // We can do this with a custom VFS that writes to both the console and USB CDC streams.
    ESP_ERROR_CHECK(tinyusb_console_init(TINYUSB_CDC_ACM_0));

    // TODO: Considering performance, do we actually want to be using USB-CDC for the logging?
    //       Minimal latency is quite critical, so we need to ensure this doesn't have any contention with the HID interface.
    //       It's just a little frustrating for debugging if we need a phyiscal UART connection to see the logs.

    // Keep the task idle until we're asked to clean up.
    vTaskSuspend(NULL);

    ESP_ERROR_CHECK(tinyusb_console_deinit(TINYUSB_CDC_ACM_0));

    ESP_ERROR_CHECK(tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0));

    ESP_ERROR_CHECK(tinyusb_driver_uninstall());

    vTaskDelete(NULL);
}