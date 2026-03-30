#pragma once

#include <stdint.h>

#include "tinyusb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Our custom report data structures
// All reports are padded to 63 bytes (CFG_TUD_HID_EP_BUFSIZE - 1)
// This seems to only strictly be necessary for IN reports, but apply it everywhere

// DeviceInfo (Device -> Host, Feature Report 1 via GET_REPORT)
typedef struct {
    uint8_t firmware_version_major;
    uint8_t firmware_version_minor;
    uint8_t features;
    uint8_t padding[60];
} __attribute__((packed)) hid_device_info_feature_report_t;

_Static_assert(
    sizeof(hid_device_info_feature_report_t) == (CFG_TUD_HID_EP_BUFSIZE - 1),
    "HID device info feature report wrong size"
);

// ScreenSpec (Host -> Device, OUT Report 1)
typedef struct {
    uint8_t screen_id;
    uint8_t frag_total;
    uint8_t frag_index;
    uint8_t payload[60];
} __attribute__((packed)) hid_screen_spec_fragment_out_report_t;

_Static_assert(
    sizeof(hid_screen_spec_fragment_out_report_t) == (CFG_TUD_HID_EP_BUFSIZE - 1),
    "HID screen spec fragment out report wrong size"
);

// VariableUpdate (Host -> Device, OUT Report 2)
typedef struct {
    uint8_t count;
    uint8_t entries[62];
} __attribute__((packed)) hid_variable_update_out_report_t;

_Static_assert(
    sizeof(hid_variable_update_out_report_t) == (CFG_TUD_HID_EP_BUFSIZE - 1),
    "HID variable update out report wrong size"
);

typedef struct {
    uint8_t event_type;
    uint8_t event_data;
} __attribute__((packed)) hid_input_report_event_t;

// InputReport (Device -> Host, IN Report 1)
typedef struct {
    uint8_t active_screen_id;
    int8_t encoder_deltas[4];
    uint8_t event_count;
    hid_input_report_event_t events[28];
    uint8_t padding[1];
} __attribute__((packed)) hid_input_report_t;

_Static_assert(sizeof(hid_input_report_t) == (CFG_TUD_HID_EP_BUFSIZE - 1), "HID input report wrong size");

// Application callbacks

void on_usb_hid_screen_spec_fragment_report(const hid_screen_spec_fragment_out_report_t *report);

void on_usb_hid_variable_update_report(const hid_variable_update_out_report_t *report);

// Function prototypes

void usb_task_main(void *pvParameters);

void set_usb_hid_device_info(uint8_t firmware_version_major, uint8_t firmware_version_minor, uint8_t features);

void send_usb_hid_input_report(const hid_input_report_t *report);

#ifdef __cplusplus
}
#endif
