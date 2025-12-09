#pragma once

#include <stdint.h>

#include "tinyusb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Our custom report data structures

typedef struct {
    int32_t stroke_limit;
    int32_t velocity_limit;
    int32_t acceleration_limit;
} __attribute__((packed)) hid_options_feature_report_t;

typedef struct {
    uint16_t status_flags;
    uint8_t sub_state;
    uint8_t main_state;
    int32_t actual_position;
    int32_t demand_position;
    int16_t current;
    uint16_t warning_flags;
    uint16_t error_code;
} __attribute__((packed)) hid_status_input_report_t;

typedef struct {
    uint8_t enabled : 1;
    uint8_t stopped : 1;
    uint8_t reserved : 6;
    int32_t start_position;
    int32_t end_position;
    int32_t direction_change_tolerance;
    int32_t forwards_velocity;
    int32_t forwards_acceleration;
    int32_t forwards_deceleration;
    int32_t backwards_velocity;
    int32_t backwards_acceleration;
    int32_t backwards_deceleration;
    // Padding to make report size 63 bytes, without this, reports don't arrive at our host program
    uint8_t padding[26];
} __attribute__((packed)) hid_control_output_report_t;

_Static_assert(sizeof(hid_options_feature_report_t) <= 63, "HID options feature report too large");
_Static_assert(sizeof(hid_status_input_report_t) <= 63, "HID status input report too large");
_Static_assert(sizeof(hid_control_output_report_t) == 63, "HID control output report wrong size");

// Application callbacks

void on_usb_hid_options_report(const hid_options_feature_report_t *report);

void on_usb_hid_status_report(const hid_status_input_report_t *report);

// Function prototypes

void usb_task_main(void *pvParameters);

void send_usb_hid_control_report(const hid_control_output_report_t *report);

#ifdef __cplusplus
}
#endif
