#include "state.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "globals.h"
#include "usb.h"

static const char *TAG = "StateMgr";

SemaphoreHandle_t input_report_mutex = NULL;
hid_input_report_t current_input_report = {};
static bool encoder_long_press_active[ENCODER_COUNT] = {false};

static const uint8_t SCREEN_TYPE_TEXT_LINES = 0;
static const uint8_t SCREEN_TYPE_MENU = 1;

// ScreenSpec limits
#define MAX_SCREEN_LINES 6
#define MAX_MENU_ITEMS   63

// VariableUpdate limits
#define MAX_VAR_INDEX    32 // No support for extended format yet
#define STRING_POOL_SIZE 4096

#define MAX_EVENT_COUNT 28

typedef struct {
    uint8_t item_id;
    uint8_t flags;
    char *label; /* pointer into spec buffer */
} menu_item_ptr_t;

typedef struct {
    uint8_t screen_id;
    /* encoder labels (pointers into spec buffer) */
    char *encoder_primary[ENCODER_COUNT];
    char *encoder_secondary[ENCODER_COUNT];

    /* left main area */
    uint8_t left_top_margin;
    uint8_t left_type;
    uint8_t left_line_count;
    char *left_lines[MAX_SCREEN_LINES];
    char *left_menu_title;
    uint8_t left_menu_item_count;
    menu_item_ptr_t left_menu_items[MAX_MENU_ITEMS];

    /* right main area */
    uint8_t right_top_margin;
    uint8_t right_type;
    uint8_t right_line_count;
    char *right_lines[MAX_SCREEN_LINES];
    char *right_menu_title;
    uint8_t right_menu_item_count;
    menu_item_ptr_t right_menu_items[MAX_MENU_ITEMS];
} screen_spec_t;

static char string_pool[STRING_POOL_SIZE];
static size_t string_pool_head = 0;

typedef enum { VAR_NONE = 0, VAR_FIXEDPOINT = 1, VAR_SHORTSTRING = 2 } var_type_t;

typedef struct {
    uint32_t seq; /* bump on each update */
    var_type_t type;
    union {
        struct {
            uint8_t decimals;
            int16_t value;
        } fp;
        char *str; /* pointer into string pool */
    } u;
} variable_value_t;

static variable_value_t var_table[MAX_VAR_INDEX];

// Label binding registry: map LVGL labels to their template and referenced var mask
typedef struct {
    lv_obj_t *label;
    const char *tmpl;  /* pointer into current_spec_buffer */
    uint32_t var_mask; /* bit i set if template references variable i */
    size_t buf_size;   /* render buffer size */
    char *buf;         /* allocated render buffer */
    int8_t encoder_index; /* -1 if not tied to an encoder */
    int8_t encoder_y_offset; /* y-offset used when aligning encoder labels */
} label_binding_t;

static label_binding_t *label_bindings = NULL;
static size_t label_binding_count = 0;

/* Assembly state (ordered fragments starting at 0) */
static uint8_t *assembling_buffer = NULL;
static int assembling_screen_id = -1;
static uint8_t assembling_frag_total = 0;
static uint8_t assembling_next_index = 0;

/* Current parsed spec: pointers reference current_spec_buffer */
static uint8_t *current_spec_buffer = NULL;
static screen_spec_t *current_parsed_screen = NULL;

/**
 * @brief Aligns an object near the bottom of the screen, adjusting its position to align with the rotary encoder
 * center.
 */
void align_encoder_center(lv_obj_t *object, int32_t y_offset, uint8_t encoder_idx) {
    static const int32_t LEFT_SCREEN_LEFT_ENCODER_OFFSET = 17;
    static const int32_t LEFT_SCREEN_RIGHT_ENCODER_OFFSET = 107;
    static const int32_t RIGHT_SCREEN_LEFT_ENCODER_OFFSET = 21;
    static const int32_t RIGHT_SCREEN_RIGHT_ENCODER_OFFSET = 111;

    bool right_screen = (encoder_idx >= ENCODER_COUNT / 2);
    bool right_encoder = (encoder_idx % 2 == 1);

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

/* Clear and free all label bindings (call when screen is replaced) */
static void clear_label_bindings(void) {
    if (!label_bindings)
        return;
    for (size_t i = 0; i < label_binding_count; ++i) {
        free(label_bindings[i].buf);
        label_bindings[i].buf = NULL;
    }
    free(label_bindings);
    label_bindings = NULL;
    label_binding_count = 0;
}

/* Register a label for future variable-driven updates. If tmpl contains no variables, registration is skipped. */
/* encoder_index: -1 for non-encoder labels; encoder_y_offset ignored when -1 */
static void register_label_binding(lv_obj_t *label, const char *tmpl, size_t buf_size, int8_t encoder_index, int8_t encoder_y_offset) {
    if (!tmpl || tmpl[0] == '\0' || !label)
        return;

    uint32_t mask = 0;
    const char *s = tmpl;
    while (*s) {
        if (*s == '{') {
            const char *q = s + 1;
            int idx = 0;
            bool found = false;
            while (*q >= '0' && *q <= '9') {
                idx = idx * 10 + (*q - '0');
                q++;
            }
            if (*q == '}')
                found = true;
            if (found && idx >= 0 && idx < (int)MAX_VAR_INDEX) {
                mask |= (1u << idx);
            }
            if (found)
                s = q + 1;
            else
                s++;
        } else {
            s++;
        }
    }

    if (mask == 0)
        return; /* no vars referenced -> no need to re-render */

    label_bindings = realloc(label_bindings, sizeof(label_binding_t) * (label_binding_count + 1));
    if (!label_bindings)
        return;
    label_bindings[label_binding_count].label = label;
    label_bindings[label_binding_count].tmpl = tmpl;
    label_bindings[label_binding_count].var_mask = mask;
    label_bindings[label_binding_count].buf_size = buf_size;
    label_bindings[label_binding_count].buf = malloc(buf_size);
    label_bindings[label_binding_count].encoder_index = encoder_index;
    label_bindings[label_binding_count].encoder_y_offset = encoder_y_offset;
    if (label_bindings[label_binding_count].buf)
        label_bindings[label_binding_count].buf[0] = '\0';
    label_binding_count++;
}

/* Ensure label bindings cleared when current parsed screen is freed */
static void free_current_parsed_screen(void) {
    if (current_parsed_screen) {
        free(current_parsed_screen);
        current_parsed_screen = NULL;
    }
    free(current_spec_buffer);
    current_spec_buffer = NULL;

    /* remove and free any label bindings from previous screen */
    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    clear_label_bindings();
    xSemaphoreGive(lvgl_mutex);
}

/* Parse a "main area" in-place. Advances *p; writes outputs into provided arrays. */
static void parse_main_area_inplace(
    const uint8_t **p, const uint8_t *end, uint8_t *out_type, uint8_t *out_top_margin, uint8_t *out_line_count,
    char *out_lines[MAX_SCREEN_LINES], char **out_menu_title, uint8_t *out_item_count,
    menu_item_ptr_t out_items[MAX_MENU_ITEMS]
) {
    if (*p >= end) {
        *out_type = 0;
        *out_top_margin = 0;
        *out_line_count = 0;
        *out_item_count = 0;
        return;
    }
    uint8_t type = (uint8_t)*(*p)++;
    *out_type = type;

    /* Read top margin that's part of the body for TextLines and Menu */
    if (*p < end) {
        *out_top_margin = (uint8_t)*(*p)++;
    } else {
        *out_top_margin = 0;
    }

    if (type == SCREEN_TYPE_TEXT_LINES) { /* TextLines */
        if (*p >= end) {
            *out_line_count = 0;
            return;
        }
        uint8_t orig_line_count = (uint8_t)*(*p)++;
        *out_line_count = 0;
        for (uint8_t i = 0; i < orig_line_count; ++i) {
            const uint8_t *start = *p;
            while (*p < end && **p != '\0')
                (*p)++;
            if (i < MAX_SCREEN_LINES) {
                out_lines[*out_line_count] = (char *)start;
                (*out_line_count)++;
            }
            if (*p < end)
                (*p)++;
            else
                break;
        }
    } else if (type == SCREEN_TYPE_MENU) { /* Menu */
        const uint8_t *title_start = *p;
        while (*p < end && **p != '\0')
            (*p)++;
        *out_menu_title = (char *)title_start;
        if (*p < end)
            (*p)++;
        if (*p >= end) {
            *out_item_count = 0;
            return;
        }
        uint8_t orig_item_count = (uint8_t)*(*p)++;
        *out_item_count = 0;
        for (uint8_t i = 0; i < orig_item_count; ++i) {
            if ((size_t)(end - *p) < 2) {
                break;
            }
            uint8_t item_id = (uint8_t)*(*p)++;
            uint8_t flags = (uint8_t)*(*p)++;
            const uint8_t *lbl_start = *p;
            while (*p < end && **p != '\0')
                (*p)++;
            if (i < MAX_MENU_ITEMS) {
                out_items[*out_item_count].item_id = item_id;
                out_items[*out_item_count].flags = flags;
                out_items[*out_item_count].label = (char *)lbl_start;
                (*out_item_count)++;
            }
            if (*p < end)
                (*p)++;
            else
                break;
        }
    } else {
        /* Unknown/future type: no-op (leave counts zero) */
        *out_line_count = 0;
        *out_item_count = 0;
    }
}

/* Parse entire reassembled payload in-place. Returns heap-allocated screen_spec_t (pointers into buf). */
static screen_spec_t *parse_screen_spec_inplace(uint8_t screen_id, uint8_t *buf, size_t size) {
    const uint8_t *p = buf;
    const uint8_t *end = buf + size;
    screen_spec_t *spec = (screen_spec_t *)calloc(1, sizeof(screen_spec_t));
    spec->screen_id = screen_id;

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        const uint8_t *start = p;
        while (p < end && *p != '\0')
            p++;
        spec->encoder_primary[i] = (char *)start;
        if (p < end)
            p++;
        else {
            spec->encoder_primary[i] = (char *)start;
            break;
        }

        start = p;
        while (p < end && *p != '\0')
            p++;
        spec->encoder_secondary[i] = (char *)start;
        if (p < end)
            p++;
        else {
            spec->encoder_secondary[i] = (char *)start;
            break;
        }
    }

    parse_main_area_inplace(
        &p, end, &spec->left_type, &spec->left_top_margin, &spec->left_line_count, spec->left_lines,
        &spec->left_menu_title, &spec->left_menu_item_count, spec->left_menu_items
    );

    parse_main_area_inplace(
        &p, end, &spec->right_type, &spec->right_top_margin, &spec->right_line_count, spec->right_lines,
        &spec->right_menu_title, &spec->right_menu_item_count, spec->right_menu_items
    );

    return spec;
}

static void log_screen_spec(const screen_spec_t *spec) {
    if (!spec)
        return;

    ESP_LOGD(TAG, "ScreenSpec id=%d", spec->screen_id);

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        ESP_LOGD(
            TAG, "  Encoder %d primary='%s' secondary='%s'", i,
            spec->encoder_primary[i] ? spec->encoder_primary[i] : "",
            spec->encoder_secondary[i] ? spec->encoder_secondary[i] : ""
        );
    }

    ESP_LOGD(TAG, "  Left type=%d", spec->left_type);
    ESP_LOGD(TAG, "    Left top_margin=%d", spec->left_top_margin);
    if (spec->left_type == SCREEN_TYPE_TEXT_LINES) {
        for (int i = 0; i < spec->left_line_count; ++i) {
            ESP_LOGD(TAG, "    Left line %d: '%s'", i, spec->left_lines[i] ? spec->left_lines[i] : "");
        }
    } else if (spec->left_type == SCREEN_TYPE_MENU) {
        ESP_LOGD(TAG, "    Title: '%s'", spec->left_menu_title ? spec->left_menu_title : "");
        for (int i = 0; i < spec->left_menu_item_count; ++i) {
            ESP_LOGD(
                TAG, "    Item %d id=%d flags=0x%02x label='%s'", i, spec->left_menu_items[i].item_id,
                spec->left_menu_items[i].flags, spec->left_menu_items[i].label ? spec->left_menu_items[i].label : ""
            );
        }
    }

    ESP_LOGD(TAG, "  Right type=%d", spec->right_type);
    ESP_LOGD(TAG, "    Right top_margin=%d", spec->right_top_margin);
    if (spec->right_type == SCREEN_TYPE_TEXT_LINES) {
        for (int i = 0; i < spec->right_line_count; ++i) {
            ESP_LOGD(TAG, "    Right line %d: '%s'", i, spec->right_lines[i] ? spec->right_lines[i] : "");
        }
    } else if (spec->right_type == SCREEN_TYPE_MENU) {
        ESP_LOGD(TAG, "    Title: '%s'", spec->right_menu_title ? spec->right_menu_title : "");
        for (int i = 0; i < spec->right_menu_item_count; ++i) {
            ESP_LOGD(
                TAG, "    Item %d id=%d flags=0x%02x label='%s'", i, spec->right_menu_items[i].item_id,
                spec->right_menu_items[i].flags, spec->right_menu_items[i].label ? spec->right_menu_items[i].label : ""
            );
        }
    }
}

/* Render a template string with {N} placeholders into buf.
   Template pointer is into current_spec_buffer (null-terminated).
   Returns required length (may exceed buf_size). */
static size_t render_template_to_buf(const char *tmpl, char *buf, size_t buf_size) {
    const char *s = tmpl;
    size_t out = 0;
    char tmp[64];

    while (*s) {
        if (*s == '{') {
            /* parse number until '}' */
            const char *q = s + 1;
            int idx = 0;
            if (*q == '\0') { /* stray '{' */
                q = s;
            } else {
                while (*q >= '0' && *q <= '9') {
                    idx = idx * 10 + (*q - '0');
                    q++;
                }
                if (*q == '}') {
                    /* substitute variable idx */
                    if ((unsigned)idx < MAX_VAR_INDEX && var_table[idx].type != VAR_NONE) {
                        if (var_table[idx].type == VAR_SHORTSTRING && var_table[idx].u.str) {
                            size_t l = strlen(var_table[idx].u.str);
                            if (out + l < buf_size)
                                memcpy(buf + out, var_table[idx].u.str, l);
                            out += l;
                        } else if (var_table[idx].type == VAR_FIXEDPOINT) {
                            int16_t v = var_table[idx].u.fp.value;
                            uint8_t dec = var_table[idx].u.fp.decimals;
                            if (dec == 0) {
                                int rv = snprintf(tmp, sizeof(tmp), "%d", (int)v);
                                if (rv > 0) {
                                    size_t l = (size_t)rv;
                                    if (out + l < buf_size)
                                        memcpy(buf + out, tmp, l);
                                    out += l;
                                }
                            } else {
                                int sign = v < 0 ? -1 : 1;
                                int32_t absv = v * sign;
                                int32_t scale = 1;
                                for (uint8_t d = 0; d < dec; ++d)
                                    scale *= 10;
                                int32_t intpart = absv / scale;
                                int32_t frac = absv % scale;
                                if (sign < 0)
                                    intpart = -intpart;
                                int rv = snprintf(tmp, sizeof(tmp), "%d.%0*d", (int)intpart, dec, (int)frac);
                                if (rv > 0) {
                                    size_t l = (size_t)rv;
                                    if (out + l < buf_size)
                                        memcpy(buf + out, tmp, l);
                                    out += l;
                                }
                            }
                        }
                    }
                    s = q + 1;
                    continue;
                }
            }
            /* if we reach here, treat '{' as literal */
            if (out + 1 < buf_size)
                buf[out] = *s;
            out++;
            s++;
        } else {
            if (out + 1 < buf_size)
                buf[out] = *s;
            out++;
            s++;
        }
    }

    if (buf_size > 0) {
        size_t to_write = (out < buf_size - 1) ? out : (buf_size - 1);
        buf[to_write] = '\0';
    }
    return out;
}

/* Re-render and update all labels whose var_mask intersects dirty_mask.
   Caller must hold lvgl_mutex. */
static void update_bound_labels(uint32_t dirty_mask) {
    if (dirty_mask == 0 || label_binding_count == 0)
        return;

    for (size_t i = 0; i < label_binding_count; ++i) {
        label_binding_t *b = &label_bindings[i];
        if ((b->var_mask & dirty_mask) == 0)
            continue;
        if (!b->buf || !b->tmpl || !b->label)
            continue;
        render_template_to_buf(b->tmpl, b->buf, b->buf_size);
        lv_label_set_text(b->label, b->buf);
        /* If this label is tied to an encoder, re-run the encoder alignment
           so position updates to reflect the new text width. */
        if (b->encoder_index >= 0) {
            lv_obj_update_layout(b->label);
            align_encoder_center(b->label, b->encoder_y_offset, (uint8_t)b->encoder_index);
        }
    }
}

static void display_screen_spec_side(const screen_spec_t *spec, bool right_side) {
    int display_index = right_side ? 1 : 0;
    lv_display_set_default(display_handles[display_index]);

    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    int enc_start = right_side ? (ENCODER_COUNT / 2) : 0;
    int enc_end = right_side ? ENCODER_COUNT : (ENCODER_COUNT / 2);

    for (int enc = enc_start; enc < enc_end; ++enc) {
        lv_obj_t *rot_label = lv_label_create(screen);
        const char *tmpl = (spec->encoder_primary[enc]) ? spec->encoder_primary[enc] : "";
        char tmp[64];
        render_template_to_buf(tmpl, tmp, sizeof(tmp));
        lv_label_set_text(rot_label, tmp);
        register_label_binding(rot_label, tmpl, sizeof(tmp), (int8_t)enc, 0);
        align_encoder_center(rot_label, 0, enc);

        lv_obj_t *btn_label = lv_label_create(screen);
        const char *tmpl2 = (spec->encoder_secondary[enc]) ? spec->encoder_secondary[enc] : "";
        char tmp2[64];
        render_template_to_buf(tmpl2, tmp2, sizeof(tmp2));
        lv_label_set_text(btn_label, tmp2);
        register_label_binding(btn_label, tmpl2, sizeof(tmp2), (int8_t)enc, -10);
        align_encoder_center(btn_label, -10, enc);
    }

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_style_all(container);

    uint8_t type = right_side ? spec->right_type : spec->left_type;
    uint8_t line_count = right_side ? spec->right_line_count : spec->left_line_count;
    char *const *text_lines = right_side ? spec->right_lines : spec->left_lines;
    char *menu_title = right_side ? spec->right_menu_title : spec->left_menu_title;
    uint8_t item_count = right_side ? spec->right_menu_item_count : spec->left_menu_item_count;
    const menu_item_ptr_t *menu_items = right_side ? spec->right_menu_items : spec->left_menu_items;

    if (type == SCREEN_TYPE_TEXT_LINES) {
        for (int i = 0; i < line_count; ++i) {
            if (!text_lines[i] || text_lines[i][0] == '\0')
                continue;

            lv_obj_t *line = lv_label_create(container);

            char linebuf[128];
            render_template_to_buf(text_lines[i], linebuf, sizeof(linebuf));
            lv_label_set_text(line, linebuf);
            register_label_binding(line, text_lines[i], sizeof(linebuf), -1, 0);

            lv_obj_align(line, LV_ALIGN_TOP_MID, 0, i * 11);
        }
    } else if (type == SCREEN_TYPE_MENU) {
        if (menu_title) {
            lv_obj_t *title = lv_label_create(container);

            char linebuf[128];
            render_template_to_buf(menu_title, linebuf, sizeof(linebuf));
            lv_label_set_text(title, linebuf);
            register_label_binding(title, menu_title, sizeof(linebuf), -1, 0);

            lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
        }

        // TODO: Make this a real menu with selectable items
        for (int i = 0; i < item_count; ++i) {
            if (!menu_items[i].label || menu_items[i].label[0] == '\0')
                continue;

            lv_obj_t *item = lv_label_create(container);

            char linebuf[128];
            render_template_to_buf(menu_items[i].label, linebuf, sizeof(linebuf));
            lv_label_set_text(item, linebuf);
            register_label_binding(item, menu_items[i].label, sizeof(linebuf), -1, 0);

            lv_obj_align(item, LV_ALIGN_TOP_MID, 0, (11 + 4) + i * 11);
        }
    }

    uint8_t top_margin = right_side ? spec->right_top_margin : spec->left_top_margin;
    lv_obj_align(container, LV_ALIGN_TOP_MID, 0, top_margin);

    lv_screen_load_anim(screen, LV_SCREEN_LOAD_ANIM_MOVE_TOP, 300, 0, true);
}

static void display_screen_spec(const screen_spec_t *spec) {
    if (!spec)
        return;

    log_screen_spec(spec);

    /* Left display */
    display_screen_spec_side(spec, false);

    /* Right display */
    display_screen_spec_side(spec, true);
}

void set_led_ring_value(uint8_t ring_index, uint8_t value) {
    if (ring_index >= ENCODER_COUNT) {
        ESP_LOGW(TAG, "Attempted to set LED ring value for invalid ring index %d", ring_index);
        return;
    }

    for (int i = 0; i < IS31FL3746A_LED_COUNT; ++i) {
        uint32_t lhs = (uint32_t)value * (uint32_t)IS31FL3746A_LED_COUNT;
        uint32_t top = UINT8_MAX * (uint32_t)(i + 1);
        uint32_t bottom = UINT8_MAX * (uint32_t)i;

        uint8_t brightness = 0;
        if (lhs >= top) {
            brightness = UINT8_MAX;
        } else if (lhs > bottom) {
            uint32_t numerator = lhs - bottom; // in [1..254*]
            uint32_t add = (numerator * 102u) / UINT8_MAX;
            uint32_t tmp = 128u + add;
            brightness = (tmp <= UINT8_MAX) ? (uint8_t)tmp : UINT8_MAX;
        }

        is31fl3746a_set_led_color(ring_handles[ring_index], i, brightness, brightness, brightness);
    }

    is31fl3746a_flush_led_color(ring_handles[ring_index]);
}

static char *string_pool_alloc_copy(const char *src, size_t len) {
    if (len + 1 > STRING_POOL_SIZE)
        return NULL; /* too large single string */
    /* wrap simple ring */
    if (string_pool_head + len + 1 > STRING_POOL_SIZE) {
        string_pool_head = 0;
    }
    char *dst = &string_pool[string_pool_head];
    memcpy(dst, src, len);
    dst[len] = '\0';
    string_pool_head += len + 1;
    return dst;
}

/* Helper store functions */
static void store_fixedpoint_var(uint8_t index, uint8_t decimals, int16_t value) {
    if (index >= MAX_VAR_INDEX)
        return;
    variable_value_t *v = &var_table[index];
    v->type = VAR_FIXEDPOINT;
    v->u.fp.decimals = decimals;
    v->u.fp.value = value;
    v->seq++;
}

static size_t store_shortstring_var(uint8_t index, const char *str) {
    size_t len = strlen(str);
    if (index >= MAX_VAR_INDEX)
        return len;
        
    /* Special-case empty string to avoid consuming pool space */
    variable_value_t *v = &var_table[index];
    if (len == 0) {
        static const char empty_string[] = "";
        v->type = VAR_SHORTSTRING;
        v->u.str = (char *)empty_string;
        v->seq++;
        return len;
    }

    char *copied = string_pool_alloc_copy(str, len);
    if (!copied) {
        /* string too long for pool: ignore update */
        return len;
    }

    v->type = VAR_SHORTSTRING;
    v->u.str = copied;
    v->seq++;
    return len;
}

static void handle_hardware_control(uint8_t index, uint8_t value) {
    /* LED rings (0-3), Reset (4), Sleep (5), others reserved */
    if (index <= 3) {
        set_led_ring_value(index, value);
    } else if (index == 4 && value == 1) {
        ESP_LOGI(TAG, "Resetting due to request from HID variable update");
        esp_restart();
    } else if (index == 5) {
        ESP_LOGW(TAG, "Received unimplemented sleep level update (value %d)", value);
        /* ignore for now */
    } else {
        ESP_LOGW(TAG, "Received hardware control update with unknown index %d (value %d)", index, value);
        /* ignore for now */
    }
}

/* on_usb_hid_screen_spec_fragment_report:
   - Assumes fragments start at index 0 and arrive in-order.
   - If a fragment 0 arrives, start/restart assembly.
   - If any out-of-order fragment arrives, ignore until fragment 0 restarts the sequence.
*/
void on_usb_hid_screen_spec_fragment_report(const hid_screen_spec_fragment_out_report_t *report) {
    ESP_LOGI(
        TAG, "Received HID screen spec fragment report: %d/%d for screen ID %d", report->frag_index + 1,
        report->frag_total, report->screen_id
    );

    /* Restart on frag 0 */
    if (report->frag_index == 0) {
        free(assembling_buffer);
        assembling_buffer = NULL;
        assembling_screen_id = report->screen_id;
        assembling_frag_total = report->frag_total;
        assembling_next_index = 0;
        size_t total_size = (size_t)assembling_frag_total * sizeof(report->payload);
        assembling_buffer = (uint8_t *)calloc(1, total_size);
        if (!assembling_buffer) {
            ESP_LOGE(TAG, "OOM allocating screen spec buffer size=%d", (int)total_size);
            assembling_screen_id = -1;
            assembling_frag_total = 0;
            return;
        }
    }

    if (assembling_buffer == NULL || report->screen_id != assembling_screen_id) {
        ESP_LOGW(TAG, "Ignoring fragment %d for screen %d (no active assembly)", report->frag_index, report->screen_id);
        return;
    }

    if (report->frag_total != assembling_frag_total) {
        ESP_LOGW(
            TAG, "Fragment total %d does not match expected %d — waiting for restart at frag 0", report->frag_total,
            assembling_frag_total
        );
        return;
    }

    /* Expect fragments in-order */
    if (report->frag_index != assembling_next_index) {
        ESP_LOGW(
            TAG, "Out-of-order fragment %d (expected %d) — waiting for restart at frag 0", report->frag_index,
            assembling_next_index
        );
        return;
    }

    /* Copy payload into slot */
    size_t slot_offset = (size_t)report->frag_index * sizeof(report->payload);
    memcpy(assembling_buffer + slot_offset, report->payload, sizeof(report->payload));
    assembling_next_index++;

    /* If not complete yet, wait for more fragments */
    if (assembling_next_index < assembling_frag_total)
        return;

    /* Complete: take ownership of buffer as current_spec_buffer */
    size_t total_size = (size_t)assembling_frag_total * sizeof(report->payload);
    uint8_t *buf = assembling_buffer;
    assembling_buffer = NULL;
    assembling_screen_id = -1;
    assembling_frag_total = 0;
    assembling_next_index = 0;

    /* Parse in-place; spec points into buf */
    screen_spec_t *spec = parse_screen_spec_inplace(report->screen_id, buf, total_size);

    /* Replace current parsed screen and buffer (free previous) */
    free_current_parsed_screen();
    current_parsed_screen = spec;
    current_spec_buffer = buf;

    /* 1) Update LVGL under lvgl_mutex */
    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    display_screen_spec(spec);
    xSemaphoreGive(lvgl_mutex);

    /* 2) Notify host that we've processed the screen */
    xSemaphoreTake(input_report_mutex, portMAX_DELAY);
    current_input_report.active_screen_id = report->screen_id;
    xSemaphoreGive(input_report_mutex);
}

void on_usb_hid_variable_update_report(const hid_variable_update_out_report_t *report) {
    ESP_LOGI(TAG, "Received HID variable update report: count=%d", report->count);

    // TODO: We should really be defensive about bounds checks here...

    uint32_t dirty_mask = 0;

    const uint8_t *entries = report->entries;
    for (uint8_t i = 0; i < report->count; ++i) {
        uint8_t tag = *entries++;

        // Compact format tag bits, parse this up-front
        uint8_t type = (tag & 0x60) >> 5;
        uint8_t index = tag & 0x1F;

        if (tag & 0x80) {
            // Extended format, not currently used
            uint8_t index = *entries++;
            uint8_t len = *entries++;
            entries += len;
            ESP_LOGW(TAG, "Unsupported extended variable update: type=%d index=%d len=%d", tag & 0x7F, index, len);
        } else if (type == 0) {
            uint8_t decimals = *entries++;
            uint16_t value = *entries++;
            value |= *entries++ << 8;
            ESP_LOGD(TAG, "Variable update: index=%d type=FixedPoint decimals=%d value=%d", index, decimals, value);
            store_fixedpoint_var(index, decimals, value);
            dirty_mask |= (1u << index);
        } else if (type == 1) {
            ESP_LOGD(TAG, "Variable update: index=%d type=ShortString value='%s'", index, entries);
            size_t len = store_shortstring_var(index, (char *)entries);
            entries += len + 1;
            dirty_mask |= (1u << index);
        } else if (type == 2) {
            ESP_LOGW(TAG, "Received variable update with reserved type 2, index %d", index);
        } else if (type == 3) {
            uint8_t value = *entries++;
            ESP_LOGD(TAG, "Variable update: index=%d type=HardwareControl value=%d", index, value);
            handle_hardware_control(index, value);
        }
    }

    if (dirty_mask) {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        update_bound_labels(dirty_mask);
        xSemaphoreGive(lvgl_mutex);
    }
}

void on_encoder_event(lv_event_t *event) {
    uint8_t index = (uint8_t)(intptr_t)lv_event_get_user_data(event);
    if (index >= ENCODER_COUNT) {
        ESP_LOGW(TAG, "Received encoder event with invalid index %d", index);
        return;
    }

    uint32_t event_code = lv_event_get_code(event);

    xSemaphoreTake(input_report_mutex, portMAX_DELAY);

    while (current_input_report.event_count >= (MAX_EVENT_COUNT - 1)) {
        ESP_LOGW(TAG, "Input report event buffer full, overwriting last event");
        current_input_report.event_count -= 1;
    }

    switch (event_code) {
    case LV_EVENT_SINGLE_CLICKED:
        ESP_LOGD(TAG, "Encoder %d single clicked", index);
        current_input_report.events[current_input_report.event_count].event_type = 0x00;
        current_input_report.events[current_input_report.event_count].event_data = index;
        current_input_report.event_count += 1;
        break;
    case LV_EVENT_DOUBLE_CLICKED:
        ESP_LOGD(TAG, "Encoder %d double clicked", index);
        current_input_report.events[current_input_report.event_count].event_type = 0x01;
        current_input_report.events[current_input_report.event_count].event_data = index;
        current_input_report.event_count += 1;
        break;
    case LV_EVENT_LONG_PRESSED:
        ESP_LOGD(TAG, "Encoder %d long press started", index);
        encoder_long_press_active[index] = true;
        current_input_report.events[current_input_report.event_count].event_type = 0x02;
        current_input_report.events[current_input_report.event_count].event_data = index;
        current_input_report.event_count += 1;
        break;
    case LV_EVENT_RELEASED:
        if (encoder_long_press_active[index]) {
            ESP_LOGD(TAG, "Encoder %d long press ended", index);
            current_input_report.events[current_input_report.event_count].event_type = 0x03;
            current_input_report.events[current_input_report.event_count].event_data = index;
            current_input_report.event_count += 1;
            encoder_long_press_active[index] = false;
        }
        break;
    case LV_EVENT_ROTARY:
        int32_t rotation = lv_event_get_rotary_diff(event);
        ESP_LOGD(TAG, "Encoder %d rotated by %d", index, rotation);
        current_input_report.encoder_deltas[index] += (int8_t)rotation;
        break;
    default:
        ESP_LOGW(TAG, "Received unhandled encoder event code %d for encoder %d", event_code, index);
        break;
    }

    xSemaphoreGive(input_report_mutex);
}

void state_task_main(void *pvParameters) {
    input_report_mutex = xSemaphoreCreateMutex();

    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    display_splash_screens();
    xSemaphoreGive(lvgl_mutex);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Set up the LEDs
    // Encoders get set to 100% brightness red
    // LED rings get set to lowest global brightness, and then per-LED scaling to force relative 50% brightness red
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        i2c_encoder_set_led_color(encoder_handles[i], 255, 0, 0);

        is31fl3746a_set_global_scale(ring_handles[i], 0x01);

        for (int j = 0; j < IS31FL3746A_LED_COUNT; ++j) {
            is31fl3746a_set_led_scale(ring_handles[i], j, 127, 0, 0);
        }

        is31fl3746a_flush_led_scale(ring_handles[i]);
    }

    // Setup event handlers to populate the input report
    // TODO: The spec says some of these should be suppressed while displaying a menu
    for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
        void *user_data = (void *)(intptr_t)i;
        lv_indev_add_event_cb(encoder_indevs[i], on_encoder_event, LV_EVENT_SINGLE_CLICKED, user_data);
        lv_indev_add_event_cb(encoder_indevs[i], on_encoder_event, LV_EVENT_DOUBLE_CLICKED, user_data);
        lv_indev_add_event_cb(encoder_indevs[i], on_encoder_event, LV_EVENT_LONG_PRESSED, user_data);
        lv_indev_add_event_cb(encoder_indevs[i], on_encoder_event, LV_EVENT_RELEASED, user_data);
        lv_indev_add_event_cb(encoder_indevs[i], on_encoder_event, LV_EVENT_ROTARY, user_data);
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        if (!tud_hid_ready()) {
            xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            continue;
        }

        xSemaphoreTake(input_report_mutex, portMAX_DELAY);
        send_usb_hid_input_report(&current_input_report);
        current_input_report.encoder_deltas[0] = 0;
        current_input_report.encoder_deltas[1] = 0;
        current_input_report.encoder_deltas[2] = 0;
        current_input_report.encoder_deltas[3] = 0;
        current_input_report.event_count = 0;
        xSemaphoreGive(input_report_mutex);

        // TODO: Get the polling interval from the USB HID subsystem (and make it configurable)
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}