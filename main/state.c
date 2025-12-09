#include "state.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "globals.h"
#include "mci_helpers.h"
#include "usb.h"

static const char *TAG = "StateMgr";

bool has_limits = false;
hid_options_feature_report_t current_limits = {};

bool has_status = false;
SemaphoreHandle_t status_mutex = NULL;
hid_status_input_report_t current_status = {};

SemaphoreHandle_t controls_mutex = NULL;
hid_control_output_report_t current_controls = {};

/**
 * @brief Aligns an object near the bottom of the screen, adjusting its position to align with the rotary encoder
 * center.
 */
void align_encoder_center(lv_obj_t *object, int32_t y_offset, bool right_screen, bool right_encoder) {
    static const int32_t LEFT_SCREEN_LEFT_ENCODER_OFFSET = 17;
    static const int32_t LEFT_SCREEN_RIGHT_ENCODER_OFFSET = 107;
    static const int32_t RIGHT_SCREEN_LEFT_ENCODER_OFFSET = 21;
    static const int32_t RIGHT_SCREEN_RIGHT_ENCODER_OFFSET = 111;

    if (right_encoder) {
        lv_obj_align(object, LV_ALIGN_BOTTOM_RIGHT, 0, y_offset);
    } else {
        lv_obj_align(object, LV_ALIGN_BOTTOM_LEFT, 0, y_offset);
    }

    lv_obj_update_layout(object);
    int32_t center = lv_obj_get_x(object) + (lv_obj_get_width(object) / 2);

    if (right_encoder) {
        int32_t target_center = right_screen ? RIGHT_SCREEN_RIGHT_ENCODER_OFFSET : LEFT_SCREEN_RIGHT_ENCODER_OFFSET;
        if (center > target_center) {
            lv_obj_set_x(object, target_center - center);
        }
    } else {
        int32_t target_center = right_screen ? RIGHT_SCREEN_LEFT_ENCODER_OFFSET : LEFT_SCREEN_LEFT_ENCODER_OFFSET;
        if (center < target_center) {
            lv_obj_set_x(object, target_center - center);
        }
    }
}

void display_splash_screens(void) {
    lv_obj_t *screen_left = lv_display_get_screen_active(display_handles[0]);
    lv_obj_t *label_left = lv_label_create(screen_left);
    lv_label_set_text(label_left, "Puddle");
    lv_obj_center(label_left);

    lv_obj_t *screen_right = lv_display_get_screen_active(display_handles[1]);
    lv_obj_t *label_right = lv_label_create(screen_right);
    lv_label_set_text(label_right, "Maker");
    lv_obj_center(label_right);
}

void set_initial_controls() {
    xSemaphoreTake(controls_mutex, portMAX_DELAY);
    current_controls.stopped = false;
    current_controls.start_position = 0;
    current_controls.end_position = 0;
    current_controls.direction_change_tolerance = 10000; // 1mm
    current_controls.forwards_velocity = 50000;          // 0.05 m/s
    current_controls.forwards_acceleration = 10000;      // 0.1 m/s^2
    current_controls.forwards_deceleration = current_controls.forwards_acceleration;
    current_controls.backwards_velocity = current_controls.forwards_velocity;
    current_controls.backwards_acceleration = current_controls.forwards_acceleration;
    current_controls.backwards_deceleration = current_controls.forwards_acceleration;
    xSemaphoreGive(controls_mutex);
}

void update_led_rings(uint8_t dirty_rings) {
    if (dirty_rings == 0 || !has_limits) {
        return;
    }

    // TODO: Provide these ring to value mappings externally
    xSemaphoreTake(controls_mutex, portMAX_DELAY);
    float ring_values_pct[4] = {
        current_controls.start_position / (float)current_limits.stroke_limit,
        current_controls.end_position / (float)current_limits.stroke_limit,
        current_controls.forwards_velocity / (float)current_limits.velocity_limit,
        current_controls.forwards_acceleration / (float)current_limits.acceleration_limit,
    };
    xSemaphoreGive(controls_mutex);

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        if ((dirty_rings & (1 << i)) == 0) {
            continue;
        }

        for (int j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
            float led_position = (j + 1.0f) / IS31FL3746A_LED_COUNT;
            float led_bottom = j / (float)IS31FL3746A_LED_COUNT;

            uint8_t brightness = 0;
            if (ring_values_pct[i] >= led_position) {
                brightness = 255;
            } else if (ring_values_pct[i] > led_bottom) {
                float normalized = (ring_values_pct[i] - led_bottom) / (led_position - led_bottom);

                brightness = (uint8_t)((0.5f + (normalized * 0.4f)) * 255.0f);
            }

            is31fl3746a_set_led_color(ring_handles[i], j, brightness, brightness, brightness);
        }

        is31fl3746a_flush_led_color(ring_handles[i]);
    }
}

typedef enum {
    ENC_BUTTON_ENABLE_DISABLE,
    ENC_BUTTON_FREEZE_UNFREEZE,
    ENC_BUTTON_OPEN_MENU,
    ENC_BUTTON_ZERO_POSITION,
} encoder_button_action_t;

void on_encoder_button_click(lv_event_t *event) {
    encoder_button_action_t *action = (encoder_button_action_t *)lv_event_get_user_data(event);

    switch (*action) {
    case ENC_BUTTON_ENABLE_DISABLE:
        ESP_LOGI(TAG, "Enable/Disable button clicked");
        xSemaphoreTake(controls_mutex, portMAX_DELAY);
        current_controls.enabled = !current_controls.enabled;
        xSemaphoreGive(controls_mutex);
        break;
    case ENC_BUTTON_FREEZE_UNFREEZE:
        ESP_LOGI(TAG, "Freeze/Unfreeze button clicked");
        xSemaphoreTake(controls_mutex, portMAX_DELAY);
        current_controls.stopped = !current_controls.stopped;
        xSemaphoreGive(controls_mutex);
        break;
    case ENC_BUTTON_OPEN_MENU:
        ESP_LOGI(TAG, "Restart button clicked, restarting system");
        esp_restart();
        break;
    case ENC_BUTTON_ZERO_POSITION:
        ESP_LOGI(TAG, "Zero Position button clicked");
        set_initial_controls();
        update_led_rings(0xFF);
        break;
    default:
        ESP_LOGW(TAG, "Unknown encoder button action %d", *action);
        break;
    }
}

typedef enum {
    ENC_ROTATION_START,
    ENC_ROTATION_END,
    ENC_ROTATION_VELOCITY,
    ENC_ROTATION_ACCELERATION,
} encoder_rotation_action_t;

void on_encoder_rotation(lv_event_t *event) {
    int32_t rotation = lv_event_get_rotary_diff(event);
    int32_t change = rotation * abs(rotation);

    xSemaphoreTake(controls_mutex, portMAX_DELAY);

    encoder_rotation_action_t *action = (encoder_rotation_action_t *)lv_event_get_user_data(event);

    uint8_t dirty_rings = 0;

    switch (*action) {
    case ENC_ROTATION_START:
        dirty_rings |= 1 << 0;
        current_controls.start_position += change * 10000;
        if (current_controls.start_position < 0)
            current_controls.start_position = 0;
        if (has_limits && current_controls.start_position > current_limits.stroke_limit)
            current_controls.start_position = current_limits.stroke_limit;
        if (current_controls.start_position > current_controls.end_position) {
            dirty_rings |= 1 << 1;
            current_controls.end_position = current_controls.start_position;
        };
        break;
    case ENC_ROTATION_END:
        dirty_rings |= 1 << 1;
        current_controls.end_position += change * 10000;
        if (current_controls.end_position < 0)
            current_controls.end_position = 0;
        if (current_controls.end_position < current_controls.start_position) {
            dirty_rings |= 1 << 0;
            current_controls.start_position = current_controls.end_position;
        }
        if (has_limits && current_controls.end_position > current_limits.stroke_limit)
            current_controls.end_position = current_limits.stroke_limit;
        break;
    case ENC_ROTATION_VELOCITY:
        dirty_rings |= 1 << 2;
        current_controls.forwards_velocity += change * 10000;
        if (current_controls.forwards_velocity < 0)
            current_controls.forwards_velocity = 0;
        if (has_limits && current_controls.forwards_velocity > current_limits.velocity_limit)
            current_controls.forwards_velocity = current_limits.velocity_limit;
        current_controls.backwards_velocity = current_controls.forwards_velocity;
        break;
    case ENC_ROTATION_ACCELERATION:
        dirty_rings |= 1 << 3;
        current_controls.forwards_acceleration += change * abs(rotation) * 1000;
        if (current_controls.forwards_acceleration < 0)
            current_controls.forwards_acceleration = 0;
        if (has_limits && current_controls.forwards_acceleration > current_limits.acceleration_limit)
            current_controls.forwards_acceleration = current_limits.acceleration_limit;
        current_controls.forwards_deceleration = current_controls.forwards_acceleration;
        current_controls.backwards_acceleration = current_controls.forwards_acceleration;
        current_controls.backwards_deceleration = current_controls.forwards_acceleration;
        break;
    default:
        ESP_LOGW(TAG, "Unknown encoder rotation action %d", *action);
        break;
    }

    xSemaphoreGive(controls_mutex);

    update_led_rings(dirty_rings);
}

typedef struct {
    lv_obj_t *enable_disable_label;
    lv_obj_t *freeze_unfreeze_label;
    lv_obj_t *left_info_label;
    lv_obj_t *right_info_label;
} control_ui_update_context_t;

#define SYMBOL_LEFT_ARROW  "\xE2\x97\x80"
#define SYMBOL_RIGHT_ARROW "\xE2\x96\xB6"

void update_controls_ui(lv_event_t *event) {
    control_ui_update_context_t *context = (control_ui_update_context_t *)lv_event_get_user_data(event);

    // Left screen

    xSemaphoreTake(controls_mutex, portMAX_DELAY);

    bool enabled = current_controls.enabled;
    bool stopped = current_controls.stopped;

    float start_position = current_controls.start_position / 10000.0f;
    float end_position = current_controls.end_position / 10000.0f;
    float backwards_velocity = current_controls.backwards_velocity / 1000000.0f;
    float forwards_velocity = current_controls.forwards_velocity / 1000000.0f;
    float backwards_acceleration = current_controls.backwards_acceleration / 100000.0f;
    float forwards_acceleration = current_controls.forwards_acceleration / 100000.0f;

    xSemaphoreGive(controls_mutex);

    // TODO: Scale dynamically based on magnitude
    char info_text[256];
    snprintf(
        info_text, sizeof(info_text),
        "Stroke " SYMBOL_LEFT_ARROW " %4.0f %4.0f " SYMBOL_RIGHT_ARROW "\n"
        " Speed " SYMBOL_LEFT_ARROW " %3.2f %3.2f " SYMBOL_RIGHT_ARROW "\n"
        "Accel. " SYMBOL_LEFT_ARROW " %3.2f %3.2f " SYMBOL_RIGHT_ARROW "",
        start_position, end_position,
        backwards_velocity, forwards_velocity,
        backwards_acceleration, forwards_acceleration
    );

    lv_label_set_text_static(context->enable_disable_label, enabled ? "Disable" : "Enable");
    align_encoder_center(context->enable_disable_label, -10, false, false);

    lv_label_set_text_static(context->freeze_unfreeze_label, stopped ? "Unfreeze" : "Freeze");
    align_encoder_center(context->freeze_unfreeze_label, -10, false, true);

    lv_label_set_text(context->left_info_label, info_text);
    lv_obj_align(context->left_info_label, LV_ALIGN_TOP_MID, 0, 7);

    // Right screen

    if (!has_status) {
        lv_label_set_text_static(context->right_info_label, "NOT CONNECTED");
        lv_obj_align(context->right_info_label, LV_ALIGN_TOP_MID, 0, 11);

        return;
    }

    xSemaphoreTake(status_mutex, portMAX_DELAY);

    uint8_t main_state = current_status.main_state;
    float actual_position = current_status.actual_position / 10000.0f;
    float demand_position = current_status.demand_position / 10000.0f;
    float current = current_status.current / 1000.0f;
    uint16_t warning_flags = current_status.warning_flags;
    uint16_t error_code = current_status.error_code;

    xSemaphoreGive(status_mutex);

    if (error_code != 0) {
        snprintf(info_text, sizeof(info_text), "ERROR\n%s", mci_error_code_to_name(error_code));

        lv_label_set_text(context->right_info_label, info_text);
        lv_obj_align(context->right_info_label, LV_ALIGN_TOP_MID, 0, 11);
        return;
    }

    snprintf(
        info_text, sizeof(info_text),
        "%s\n"
        "%4.0f / %4.0f mm\n",
        mci_state_to_name(main_state), actual_position, demand_position
    );

    if (warning_flags != 0) {
        format_mci_warning_flags(
            warning_flags, ", ", info_text + strlen(info_text), sizeof(info_text) - strlen(info_text)
        );
    } else {
        snprintf(info_text + strlen(info_text), sizeof(info_text) - strlen(info_text), "%02.2f A", current);
    }

    lv_label_set_text(context->right_info_label, info_text);
    lv_obj_align(context->right_info_label, LV_ALIGN_TOP_MID, 0, 11);
}

void display_main_screens(void) {
    // Left screen

    lv_display_set_default(display_handles[0]);
    lv_obj_t *left_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(left_screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *left_screen_left_rotation_label = lv_label_create(left_screen);
    lv_label_set_text(left_screen_left_rotation_label, "Start");
    align_encoder_center(left_screen_left_rotation_label, 0, false, false);

    lv_obj_t *left_screen_left_button_label = lv_label_create(left_screen);
    lv_label_set_text(left_screen_left_button_label, "Disable");
    align_encoder_center(left_screen_left_button_label, -10, false, false);

    lv_obj_t *left_screen_right_rotation_label = lv_label_create(left_screen);
    lv_label_set_text(left_screen_right_rotation_label, "End");
    align_encoder_center(left_screen_right_rotation_label, 0, false, true);

    lv_obj_t *left_screen_right_button_label = lv_label_create(left_screen);
    lv_label_set_text(left_screen_right_button_label, "Freeze");
    align_encoder_center(left_screen_right_button_label, -10, false, true);

    lv_obj_t *left_screen_info_label = lv_label_create(left_screen);
    lv_label_set_text(left_screen_info_label, "");
    lv_obj_align(left_screen_info_label, LV_ALIGN_TOP_MID, 0, 0);

    lv_screen_load_anim(left_screen, LV_SCREEN_LOAD_ANIM_MOVE_TOP, 500, 0, true);

    // Right screen

    lv_display_set_default(display_handles[1]);
    lv_obj_t *right_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(right_screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *right_screen_left_rotation_label = lv_label_create(right_screen);
    lv_label_set_text(right_screen_left_rotation_label, "Speed");
    align_encoder_center(right_screen_left_rotation_label, 0, true, false);

    lv_obj_t *right_screen_left_button_label = lv_label_create(right_screen);
    lv_label_set_text(right_screen_left_button_label, "Menu");
    align_encoder_center(right_screen_left_button_label, -10, true, false);

    lv_obj_t *right_screen_right_rotation_label = lv_label_create(right_screen);
    lv_label_set_text(right_screen_right_rotation_label, "Accel.");
    align_encoder_center(right_screen_right_rotation_label, 0, true, true);

    lv_obj_t *right_screen_right_button_label = lv_label_create(right_screen);
    lv_label_set_text(right_screen_right_button_label, "Zero");
    align_encoder_center(right_screen_right_button_label, -10, true, true);

    lv_obj_t *right_screen_info_label = lv_label_create(right_screen);
    lv_label_set_text(right_screen_info_label, "");
    lv_obj_set_style_text_align(right_screen_info_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(right_screen_info_label, LV_ALIGN_TOP_MID, 0, 0);

    lv_screen_load_anim(right_screen, LV_SCREEN_LOAD_ANIM_MOVE_TOP, 500, 0, true);

    // Update the UI whenever the screen is refreshed
    control_ui_update_context_t *context = malloc(sizeof(control_ui_update_context_t));
    context->enable_disable_label = left_screen_left_button_label;
    context->freeze_unfreeze_label = left_screen_right_button_label;
    context->left_info_label = left_screen_info_label;
    context->right_info_label = right_screen_info_label;
    lv_display_add_event_cb(display_handles[0], update_controls_ui, LV_EVENT_REFR_START, context);

    // TODO: Make these bindings configurable.
    // Encoder 1: Start Position, Enable/Disable on click
    // Encoder 2: End Position, Freeze/Unfreeze on click
    // Encoder 3: Velocity, Open Menu on click
    // Encoder 4: Acceleration, Zero Position on click

    encoder_rotation_action_t *rotation_action = malloc(sizeof(encoder_rotation_action_t));
    *rotation_action = ENC_ROTATION_START;
    lv_indev_add_event_cb(encoder_indevs[0], on_encoder_rotation, LV_EVENT_ROTARY, rotation_action);
    encoder_button_action_t *button_action = malloc(sizeof(encoder_button_action_t));
    *button_action = ENC_BUTTON_ENABLE_DISABLE;
    lv_indev_add_event_cb(encoder_indevs[0], on_encoder_button_click, LV_EVENT_CLICKED, button_action);

    rotation_action = malloc(sizeof(encoder_rotation_action_t));
    *rotation_action = ENC_ROTATION_END;
    lv_indev_add_event_cb(encoder_indevs[1], on_encoder_rotation, LV_EVENT_ROTARY, rotation_action);
    button_action = malloc(sizeof(encoder_button_action_t));
    *button_action = ENC_BUTTON_FREEZE_UNFREEZE;
    lv_indev_add_event_cb(encoder_indevs[1], on_encoder_button_click, LV_EVENT_CLICKED, button_action);

    rotation_action = malloc(sizeof(encoder_rotation_action_t));
    *rotation_action = ENC_ROTATION_VELOCITY;
    lv_indev_add_event_cb(encoder_indevs[2], on_encoder_rotation, LV_EVENT_ROTARY, rotation_action);
    button_action = malloc(sizeof(encoder_button_action_t));
    *button_action = ENC_BUTTON_OPEN_MENU;
    lv_indev_add_event_cb(encoder_indevs[2], on_encoder_button_click, LV_EVENT_CLICKED, button_action);

    rotation_action = malloc(sizeof(encoder_rotation_action_t));
    *rotation_action = ENC_ROTATION_ACCELERATION;
    lv_indev_add_event_cb(encoder_indevs[3], on_encoder_rotation, LV_EVENT_ROTARY, rotation_action);
    button_action = malloc(sizeof(encoder_button_action_t));
    *button_action = ENC_BUTTON_ZERO_POSITION;
    lv_indev_add_event_cb(encoder_indevs[3], on_encoder_button_click, LV_EVENT_CLICKED, button_action);
}

void on_usb_hid_options_report(const hid_options_feature_report_t *report) {
    ESP_LOGI(
        TAG, "Received HID options report: stroke_limit=%d, velocity_limit=%d, acceleration_limit=%d",
        report->stroke_limit, report->velocity_limit, report->acceleration_limit
    );

    memcpy(&current_limits, report, sizeof(hid_options_feature_report_t));
    has_limits = true;

    // Setup reasonable default controls
    set_initial_controls();
    update_led_rings(0xFF);
}

void on_usb_hid_status_report(const hid_status_input_report_t *report) {
    if (!report) {
        // TODO: It shouldn't be possible to return to this state after having received a valid report
        has_status = false;

        ESP_LOGW(TAG, "Received NULL HID status report");
        return;
    }

    ESP_LOGI(
        TAG,
        "Received HID status report: status_flags=0x%04X, sub_state=%d, main_state=%d, actual_position=%d, "
        "demand_position=%d, current=%d, warning_flags=0x%04X, error_code=0x%04X",
        report->status_flags, report->sub_state, report->main_state, report->actual_position, report->demand_position,
        report->current, report->warning_flags, report->error_code
    );

    xSemaphoreTake(status_mutex, portMAX_DELAY);
    memcpy(&current_status, report, sizeof(hid_status_input_report_t));
    has_status = true;
    xSemaphoreGive(status_mutex);
}

void state_task_main(void *pvParameters) {
    status_mutex = xSemaphoreCreateMutex();
    controls_mutex = xSemaphoreCreateMutex();

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    display_splash_screens();
    xSemaphoreGive(lvgl_mutex);

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < 10 && (!has_limits || !has_status); i++) {
        // TODO: If this takes too long, display something on screen
        ESP_LOGI(TAG, "Waiting for HID reports...");

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        i2c_encoder_set_led_color(encoder_handles[i], 255, 0, 0);

        is31fl3746a_set_global_scale(ring_handles[i], 0x01);

        for (int j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
            is31fl3746a_set_led_scale(ring_handles[i], j, 127, 0, 0);
        }

        is31fl3746a_flush_led_scale(ring_handles[i]);
    }

    // TODO: These are our default limits, use them if we're not connected
    //       This makes development a little easier
    if (!has_limits) {
        hid_options_feature_report_t default_limits = {
            .stroke_limit = 3600000,
            .velocity_limit = 1750000,
            .acceleration_limit = 999000,
        };

        on_usb_hid_options_report(&default_limits);
    }

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    display_main_screens();
    xSemaphoreGive(lvgl_mutex);

    // TODO: Better way to wait for animation to complete - perhaps the LV_EVENT_SCREEN_LOADED event?
    vTaskDelay(pdMS_TO_TICKS(500));

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        if (!tud_hid_ready()) {
            xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            continue;
        }

        xSemaphoreTake(controls_mutex, portMAX_DELAY);
        send_usb_hid_control_report(&current_controls);
        xSemaphoreGive(controls_mutex);

        // TODO: Get the polling interval from the USB HID subsystem (and make it configurable)
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}