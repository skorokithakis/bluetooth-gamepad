#include "hid_parser.h"
#include "hid_descriptor.h"
#include "gamepad_state.h"

#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "HID_PARSER";

#ifndef STICK_DEADZONE
// Deadzone applied to normalized stick axes (-32767..32767).
// Override via PlatformIO build flag `-DSTICK_DEADZONE=<value>`.
#define STICK_DEADZONE 3000
#endif

// Globals defined in usb_host.cpp when USB is enabled, otherwise provide stubs.
#ifdef DISABLE_USB_HOST
HidReportMapCollection g_hid_report_map;
bool g_hid_report_map_valid = false;
#endif

static void log_button_presses(uint32_t pressed_mask) {
    while (pressed_mask) {
        int bit = __builtin_ctz(pressed_mask);
        pressed_mask &= ~(1u << bit);
        ESP_LOGI(TAG, "Button bit %d pressed", bit);
    }
}

static constexpr uint32_t k_canonical_button_mask = (1u << (GAMEPAD_BUTTON_GUIDE + 1)) - 1u;

// Fixed HID "Button N" -> canonical button mapping.
// Override any of these via PlatformIO build flags if your controller differs.
#ifndef HID_BTN_NUM_X
#define HID_BTN_NUM_X 1
#endif
#ifndef HID_BTN_NUM_B
#define HID_BTN_NUM_B 2
#endif
#ifndef HID_BTN_NUM_A
#define HID_BTN_NUM_A 3
#endif
#ifndef HID_BTN_NUM_Y
#define HID_BTN_NUM_Y 4
#endif
#ifndef HID_BTN_NUM_LB
#define HID_BTN_NUM_LB 5
#endif
#ifndef HID_BTN_NUM_RB
#define HID_BTN_NUM_RB 6
#endif
#ifndef HID_BTN_NUM_L3
#define HID_BTN_NUM_L3 7
#endif
#ifndef HID_BTN_NUM_R3
#define HID_BTN_NUM_R3 8
#endif
#ifndef HID_BTN_NUM_BACK
#define HID_BTN_NUM_BACK 9
#endif
#ifndef HID_BTN_NUM_START
#define HID_BTN_NUM_START 10
#endif
#ifndef HID_BTN_NUM_GUIDE
#define HID_BTN_NUM_GUIDE 11
#endif

// Some controllers expose triggers as digital buttons (not analog axes).
// Configure which HID "Button N" numbers represent L2/R2 triggers.
// Set to 0 to disable digital trigger synthesis.
#ifndef HID_DIGITAL_TRIGGER_L_BUTTON
#define HID_DIGITAL_TRIGGER_L_BUTTON 5
#endif
#ifndef HID_DIGITAL_TRIGGER_R_BUTTON
#define HID_DIGITAL_TRIGGER_R_BUTTON 6
#endif

// Log raw HID button numbers on press transitions (helps controller-specific mapping).
#ifndef HID_LOG_RAW_BUTTONS
#define HID_LOG_RAW_BUTTONS 0
#endif

#if (HID_DIGITAL_TRIGGER_L_BUTTON != 0) && \
    (HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_A || HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_B || \
     HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_X || HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_Y || \
     HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_LB || HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_RB || \
     HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_L3 || HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_R3 || \
     HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_BACK || HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_START || \
     HID_DIGITAL_TRIGGER_L_BUTTON == HID_BTN_NUM_GUIDE)
#error "HID_DIGITAL_TRIGGER_L_BUTTON collides with a mapped HID button number"
#endif

#if (HID_DIGITAL_TRIGGER_R_BUTTON != 0) && \
    (HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_A || HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_B || \
     HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_X || HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_Y || \
     HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_LB || HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_RB || \
     HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_L3 || HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_R3 || \
     HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_BACK || HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_START || \
     HID_DIGITAL_TRIGGER_R_BUTTON == HID_BTN_NUM_GUIDE)
#error "HID_DIGITAL_TRIGGER_R_BUTTON collides with a mapped HID button number"
#endif

static uint32_t hid_button_number_to_canonical_mask(uint16_t button_number) {
    if (button_number == HID_BTN_NUM_X) return (1u << GAMEPAD_BUTTON_X);
    if (button_number == HID_BTN_NUM_B) return (1u << GAMEPAD_BUTTON_B);
    if (button_number == HID_BTN_NUM_A) return (1u << GAMEPAD_BUTTON_A);
    if (button_number == HID_BTN_NUM_Y) return (1u << GAMEPAD_BUTTON_Y);
    if (button_number == HID_BTN_NUM_LB) return (1u << GAMEPAD_BUTTON_LB);
    if (button_number == HID_BTN_NUM_RB) return (1u << GAMEPAD_BUTTON_RB);
    if (button_number == HID_BTN_NUM_BACK) return (1u << GAMEPAD_BUTTON_BACK);
    if (button_number == HID_BTN_NUM_START) return (1u << GAMEPAD_BUTTON_START);
    if (button_number == HID_BTN_NUM_L3) return (1u << GAMEPAD_BUTTON_L3);
    if (button_number == HID_BTN_NUM_R3) return (1u << GAMEPAD_BUTTON_R3);
    if (button_number == HID_BTN_NUM_GUIDE) return (1u << GAMEPAD_BUTTON_GUIDE);
    return 0;
}

static int16_t apply_stick_deadzone(int16_t value) {
    int32_t v = value;
    if (v > 32767) v = 32767;
    if (v < -32767) v = -32767;

    int32_t mag = (v < 0) ? -v : v;
    const int32_t dz = (STICK_DEADZONE < 0) ? 0 : (STICK_DEADZONE > 32767 ? 32767 : STICK_DEADZONE);
    if (mag <= dz) return 0;

    // Rescale so full range is still reachable after the deadzone.
    const int32_t scaled = (mag - dz) * 32767 / (32767 - dz);
    return (int16_t)((v < 0) ? -scaled : scaled);
}

// Normalize an axis value using its logical min/max to -32767..32767.
static int16_t normalize_axis(int32_t value, int32_t logical_min, int32_t logical_max) {
    if (logical_max <= logical_min) return 0;

    int32_t center = (logical_min + logical_max) / 2;
    int32_t half_range = (logical_max - logical_min) / 2;

    if (half_range == 0) return 0;

    int32_t delta = value - center;
    int32_t normalized = delta * 32767 / half_range;

    if (normalized > 32767) normalized = 32767;
    if (normalized < -32767) normalized = -32767;

    return apply_stick_deadzone((int16_t)normalized);
}

// Normalize a trigger value (0 at min, 32767 at max).
static int16_t normalize_trigger(int32_t value, int32_t logical_min, int32_t logical_max) {
    if (logical_max <= logical_min) return 0;

    int32_t range = logical_max - logical_min;
    int32_t normalized = (value - logical_min) * 32767 / range;

    if (normalized < 0) normalized = 0;
    if (normalized > 32767) normalized = 32767;

    return (int16_t)normalized;
}

// Convert 8-way D-pad value (0-7, 8/15 = centered) to hat direction.
static uint8_t dpad_value_to_hat(int32_t value, int32_t logical_min, int32_t logical_max) {
    // HID hat switches typically encode:
    // - 8 directions (N..NW) in a contiguous range
    // - plus an optional "null/center" state outside that range (often 8 or 15)
    if (logical_max >= logical_min) {
        const int32_t range = logical_max - logical_min + 1;
        if (range == 4 || range == 5) {
            // Some devices encode 4-way D-pad as 0..3 (with optional centered at 4):
            // 0=Up, 1=Right, 2=Down, 3=Left, (4=Centered).
            const int32_t v = value - logical_min;
            if (v < 0 || v >= range) return HAT_CENTERED;
            if (range == 5 && v == 4) return HAT_CENTERED;
            switch (v) {
                case 0: return HAT_UP;
                case 1: return HAT_RIGHT;
                case 2: return HAT_DOWN;
                case 3: return HAT_LEFT;
                default: return HAT_CENTERED;
            }
        }
        if (range == 8) {
            // Directions are logical_min..logical_min+7.
            if (value < logical_min || value > logical_max) return HAT_CENTERED;
            return (uint8_t)((value - logical_min) + 1);
        }
        if (range == 9) {
            // Common 0..8 encoding where 8 is centered.
            if (value < logical_min || value > logical_max) return HAT_CENTERED;
            if (value == logical_max) return HAT_CENTERED;
            return (uint8_t)((value - logical_min) + 1);
        }
    }

    // Fallback: treat 0..7 as directions, anything else centered.
    if (value < 0 || value > 7) return HAT_CENTERED;
    return (uint8_t)(value + 1);
}

// Parse report using the parsed HID descriptor.
static void parse_descriptor_based_report(const uint8_t* data, size_t length) {
    // Select report map (devices may use multiple report IDs).
    const HidReportMap* map = nullptr;
    const uint8_t* report_data = data;
    size_t report_length = length;

    const bool has_report_ids =
        g_hid_report_map.report_count > 0 &&
        (g_hid_report_map.report_count > 1 || g_hid_report_map.reports[0].report_id != 0);

    if (has_report_ids) {
        if (length < 1) return;
        const uint8_t report_id = data[0];
        map = hid_find_report_map(&g_hid_report_map, report_id);
        if (!map) return;
        report_data = data + 1;
        report_length = length - 1;
    } else {
        map = hid_find_report_map(&g_hid_report_map, 0);
        if (!map) return;
    }

    if (report_length < map->report_byte_length) {
        return;
    }

    if (xSemaphoreTake(g_gamepad_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    const GamepadState prev_state = g_gamepad_state;
    GamepadState next_state = prev_state;

    bool updated_left_trigger = false;
    bool updated_right_trigger = false;

#if HID_LOG_RAW_BUTTONS
    static uint32_t prev_raw_button_mask = 0;
    uint32_t raw_button_mask = 0;
#endif

    // Left stick: X and Y.
    const HidField* field_x = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_X);
    const HidField* field_y = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_Y);
    if (field_x) {
        int32_t raw = hid_extract_field_value(report_data, report_length, field_x, 0);
        next_state.left_stick_x = normalize_axis(raw, field_x->logical_min, field_x->logical_max);
    }
    if (field_y) {
        int32_t raw = hid_extract_field_value(report_data, report_length, field_y, 0);
        next_state.left_stick_y = normalize_axis(raw, field_y->logical_min, field_y->logical_max);
    }

    // Right stick: prefer Rx/Ry, fall back to Z/Rz.
    // (Many controllers use Z/Rz for triggers, and Rx/Ry for the right stick.)
    const HidField* field_z = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_Z);
    const HidField* field_rz = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_RZ);
    const HidField* field_rx = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_RX);
    const HidField* field_ry = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_RY);
    const bool using_rxry_for_right_stick = (field_rx && field_ry);
    if (using_rxry_for_right_stick) {
        int32_t raw_rx = hid_extract_field_value(report_data, report_length, field_rx, 0);
        int32_t raw_ry = hid_extract_field_value(report_data, report_length, field_ry, 0);
        next_state.right_stick_x = normalize_axis(raw_rx, field_rx->logical_min, field_rx->logical_max);
        next_state.right_stick_y = normalize_axis(raw_ry, field_ry->logical_min, field_ry->logical_max);
    } else if (field_z && field_rz) {
        int32_t raw_z = hid_extract_field_value(report_data, report_length, field_z, 0);
        int32_t raw_rz = hid_extract_field_value(report_data, report_length, field_rz, 0);
        next_state.right_stick_x = normalize_axis(raw_z, field_z->logical_min, field_z->logical_max);
        next_state.right_stick_y = normalize_axis(raw_rz, field_rz->logical_min, field_rz->logical_max);
    }

    // Triggers: check Simulation usage page for brake/throttle.
    const HidField* field_brake = hid_find_field(map, HID_USAGE_PAGE_SIMULATION, HID_USAGE_BRAKE);
    const HidField* field_throttle = hid_find_field(map, HID_USAGE_PAGE_SIMULATION, HID_USAGE_THROTTLE);
    if (field_brake) {
        int32_t raw = hid_extract_field_value(report_data, report_length, field_brake, 0);
        next_state.left_trigger = normalize_trigger(raw, field_brake->logical_min, field_brake->logical_max);
        updated_left_trigger = true;
    }
    if (field_throttle) {
        int32_t raw = hid_extract_field_value(report_data, report_length, field_throttle, 0);
        next_state.right_trigger = normalize_trigger(raw, field_throttle->logical_min, field_throttle->logical_max);
        updated_right_trigger = true;
    }

    // If the device exposes Z/Rz and we're using Rx/Ry for the right stick,
    // treat Z/Rz as triggers (common Xbox-style layout).
    if (using_rxry_for_right_stick && field_z && field_rz) {
        if (!updated_left_trigger) {
            int32_t raw = hid_extract_field_value(report_data, report_length, field_z, 0);
            next_state.left_trigger = normalize_trigger(raw, field_z->logical_min, field_z->logical_max);
            updated_left_trigger = true;
        }
        if (!updated_right_trigger) {
            int32_t raw = hid_extract_field_value(report_data, report_length, field_rz, 0);
            next_state.right_trigger = normalize_trigger(raw, field_rz->logical_min, field_rz->logical_max);
            updated_right_trigger = true;
        }
    }

    // Hat switch / D-pad.
    const HidField* field_hat = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_HAT_SWITCH);
    if (field_hat) {
        int32_t raw = hid_extract_field_value(report_data, report_length, field_hat, 0);
        next_state.dpad = dpad_value_to_hat(raw, field_hat->logical_min, field_hat->logical_max);
    }

    // Some devices use Generic Desktop usages for Start/Select instead of Button page.
    // Map these into our canonical BACK/START bits.
    const HidField* field_start = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_START);
    if (field_start) {
        int32_t pressed = hid_extract_field_value(report_data, report_length, field_start, 0);
        const uint32_t mask = (1u << GAMEPAD_BUTTON_START);
        if (pressed) next_state.buttons |= mask;
        else next_state.buttons &= ~mask;
    }
    const HidField* field_select = hid_find_field(map, HID_USAGE_PAGE_GENERIC_DESKTOP, HID_USAGE_SELECT);
    if (field_select) {
        int32_t pressed = hid_extract_field_value(report_data, report_length, field_select, 0);
        const uint32_t mask = (1u << GAMEPAD_BUTTON_BACK);
        if (pressed) next_state.buttons |= mask;
        else next_state.buttons &= ~mask;
    }

    // Buttons: update only the buttons present in this report.
    bool has_array_buttons = false;
    for (uint8_t i = 0; i < map->field_count; i++) {
        if (map->fields[i].usage_page == HID_USAGE_PAGE_BUTTON && !map->fields[i].is_variable) {
            has_array_buttons = true;
            break;
        }
    }
    if (has_array_buttons) {
        next_state.buttons &= ~k_canonical_button_mask;
    }

    // If we need to synthesize a hat from D-pad button numbers (13-16), track
    // them separately so they don't collide with canonical button bits.
    bool dpad_up = false;
    bool dpad_down = false;
    bool dpad_left = false;
    bool dpad_right = false;

    bool lt_button_pressed = false;
    bool rt_button_pressed = false;

    for (uint8_t i = 0; i < map->field_count; i++) {
        const HidField* field = &map->fields[i];
        if (field->usage_page != HID_USAGE_PAGE_BUTTON) continue;

        if (field->is_variable) {
            const uint16_t button_number = field->usage;
            int32_t pressed = hid_extract_field_value(report_data, report_length, field, 0);
#if HID_LOG_RAW_BUTTONS
            if (button_number > 0 && button_number <= 32 && pressed) {
                raw_button_mask |= (1u << (button_number - 1));
            }
#endif
            if (button_number == 13) dpad_up = pressed != 0;
            else if (button_number == 14) dpad_down = pressed != 0;
            else if (button_number == 15) dpad_left = pressed != 0;
            else if (button_number == 16) dpad_right = pressed != 0;
            else {
                if (HID_DIGITAL_TRIGGER_L_BUTTON != 0 && button_number == HID_DIGITAL_TRIGGER_L_BUTTON) {
                    lt_button_pressed = pressed != 0;
                    continue;
                }
                if (HID_DIGITAL_TRIGGER_R_BUTTON != 0 && button_number == HID_DIGITAL_TRIGGER_R_BUTTON) {
                    rt_button_pressed = pressed != 0;
                    continue;
                }

                const uint32_t mask = hid_button_number_to_canonical_mask(button_number);
                if (mask == 0) continue;
                if (pressed) next_state.buttons |= mask;
                else next_state.buttons &= ~mask;
            }
        } else {
            // Array buttons: clear all first (above), then set pressed indices.
            for (uint8_t j = 0; j < field->count; j++) {
                int32_t button_index = hid_extract_field_value(report_data, report_length, field, j);
#if HID_LOG_RAW_BUTTONS
                if (button_index > 0 && button_index <= 32) {
                    raw_button_mask |= (1u << ((uint32_t)button_index - 1u));
                }
#endif
                if (button_index == 13) dpad_up = true;
                else if (button_index == 14) dpad_down = true;
                else if (button_index == 15) dpad_left = true;
                else if (button_index == 16) dpad_right = true;
                else {
                    if (HID_DIGITAL_TRIGGER_L_BUTTON != 0 && button_index == HID_DIGITAL_TRIGGER_L_BUTTON) {
                        lt_button_pressed = true;
                        continue;
                    }
                    if (HID_DIGITAL_TRIGGER_R_BUTTON != 0 && button_index == HID_DIGITAL_TRIGGER_R_BUTTON) {
                        rt_button_pressed = true;
                        continue;
                    }

                    const uint32_t mask = hid_button_number_to_canonical_mask((uint16_t)button_index);
                    if (mask) next_state.buttons |= mask;
                }
            }
        }
    }

    // If we don't have analog triggers, synthesize them from digital trigger buttons.
    if (!updated_left_trigger && HID_DIGITAL_TRIGGER_L_BUTTON != 0) {
        next_state.left_trigger = lt_button_pressed ? 32767 : 0;
        updated_left_trigger = true;
    }
    if (!updated_right_trigger && HID_DIGITAL_TRIGGER_R_BUTTON != 0) {
        next_state.right_trigger = rt_button_pressed ? 32767 : 0;
        updated_right_trigger = true;
    }

#if HID_LOG_RAW_BUTTONS
    const uint32_t pressed_edges = raw_button_mask & ~prev_raw_button_mask;
    if (pressed_edges) {
        for (uint8_t b = 0; b < 32; b++) {
            if (pressed_edges & (1u << b)) {
                ESP_LOGI(TAG, "HID Button %u pressed", (unsigned)(b + 1));
            }
        }
    }
    prev_raw_button_mask = raw_button_mask;
#endif

    // Avoid "stuck" triggers if the device doesn't report any trigger source.
    if (!updated_left_trigger) next_state.left_trigger = 0;
    if (!updated_right_trigger) next_state.right_trigger = 0;

    // If no hat switch field exists, some controllers encode D-pad as 4 buttons
    // (commonly Buttons 13-16). Derive an 8-way hat from those.
    if (!field_hat) {
        uint8_t hat = HAT_CENTERED;
        if (dpad_up && dpad_right) hat = HAT_UP_RIGHT;
        else if (dpad_down && dpad_right) hat = HAT_DOWN_RIGHT;
        else if (dpad_down && dpad_left) hat = HAT_DOWN_LEFT;
        else if (dpad_up && dpad_left) hat = HAT_UP_LEFT;
        else if (dpad_up) hat = HAT_UP;
        else if (dpad_right) hat = HAT_RIGHT;
        else if (dpad_down) hat = HAT_DOWN;
        else if (dpad_left) hat = HAT_LEFT;

        next_state.dpad = hat;
    }

    const bool state_changed =
        prev_state.left_stick_x != next_state.left_stick_x ||
        prev_state.left_stick_y != next_state.left_stick_y ||
        prev_state.right_stick_x != next_state.right_stick_x ||
        prev_state.right_stick_y != next_state.right_stick_y ||
        prev_state.left_trigger != next_state.left_trigger ||
        prev_state.right_trigger != next_state.right_trigger ||
        prev_state.dpad != next_state.dpad ||
        prev_state.buttons != next_state.buttons;

    if (state_changed) {
        const uint32_t pressed_mask = (~prev_state.buttons) & next_state.buttons;
        g_gamepad_state = next_state;
        g_gamepad_state.changed = true;
        if (pressed_mask) {
            log_button_presses(pressed_mask);
        }
    }

    xSemaphoreGive(g_gamepad_mutex);
}

// ============================================================================
// Fallback heuristic parser for devices without valid HID descriptors.
// ============================================================================

// Normalize an unsigned 8-bit stick axis (0-255, 128=center) to -32767..32767.
static int16_t normalize_stick_axis_u8(uint8_t value) {
    const int32_t delta = (int32_t)value - 128;
    if (delta >= 0) {
        return apply_stick_deadzone((int16_t)(delta * 32767 / 127));
    }
    return apply_stick_deadzone((int16_t)(delta * 32767 / 128));
}

// Normalize an unsigned 8-bit trigger axis (0-255) to 0..32767.
static int16_t normalize_trigger_axis_u8(uint8_t value) {
    return (int16_t)((uint32_t)value * 32767 / 255);
}

// Convert 8-way D-pad value (0-7, 8/15 = centered) to hat direction.
static uint8_t dpad_value_to_hat_legacy(uint8_t value) {
    if (value > 7) return HAT_CENTERED;
    // D-pad values: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW.
    // Hat values: 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW.
    return value + 1;
}

// Some generic USB gamepads encode the D-pad as a hat value in the low nibble of
// one of the report bytes that also contains buttons. Track which byte we
// detected so we don't accidentally treat the hat nibble as buttons.
enum class GenericDpadNibbleLocation : uint8_t {
    Unknown = 0,
    Byte4LowNibble = 1,
    Byte5LowNibble = 2,
};

static GenericDpadNibbleLocation g_generic_dpad_location = GenericDpadNibbleLocation::Unknown;

static bool is_hat_nibble_value(uint8_t nibble) {
    // Common encodings:
    // - 0..7 = directions
    // - 8 or 15 = centered
    return nibble <= 8 || nibble == 0x0F;
}

// Parse generic 8-byte HID gamepad report.
// Common format for cheap USB gamepads.
static void parse_generic_8byte_report(const uint8_t* data, size_t length) {
    if (length < 6) return;

    const int16_t next_left_stick_x = normalize_stick_axis_u8(data[0]);
    const int16_t next_left_stick_y = normalize_stick_axis_u8(data[1]);
    const int16_t next_right_stick_x = normalize_stick_axis_u8(data[2]);
    const int16_t next_right_stick_y = normalize_stick_axis_u8(data[3]);

    const uint8_t nibble4 = data[4] & 0x0F;
    const uint8_t nibble5 = data[5] & 0x0F;

    // If we see an invalid hat value (9-14) in one nibble but not the other,
    // lock onto the other nibble as the hat for this device.
    if (g_generic_dpad_location == GenericDpadNibbleLocation::Unknown) {
        const bool nibble4_hatish = is_hat_nibble_value(nibble4);
        const bool nibble5_hatish = is_hat_nibble_value(nibble5);

        if (!nibble4_hatish && nibble5_hatish) {
            g_generic_dpad_location = GenericDpadNibbleLocation::Byte5LowNibble;
        } else if (nibble4_hatish && !nibble5_hatish) {
            g_generic_dpad_location = GenericDpadNibbleLocation::Byte4LowNibble;
        } else if ((nibble5 == 0x0F || nibble5 == 0x08) && (nibble4 != 0x0F && nibble4 != 0x08)) {
            // Centered is often 0x0F (or sometimes 0x08). If only one nibble
            // reports that centered value, prefer it as the hat.
            g_generic_dpad_location = GenericDpadNibbleLocation::Byte5LowNibble;
        } else if ((nibble4 == 0x0F || nibble4 == 0x08) && (nibble5 != 0x0F && nibble5 != 0x08)) {
            g_generic_dpad_location = GenericDpadNibbleLocation::Byte4LowNibble;
        }
    }

    const uint8_t dpad_raw =
        (g_generic_dpad_location == GenericDpadNibbleLocation::Byte5LowNibble) ? nibble5 : nibble4;
    const uint8_t next_dpad = dpad_value_to_hat_legacy(dpad_raw);

    uint32_t next_buttons = 0;
    if (length > 5) {
        uint8_t buttons_byte0 = data[5];
        if (g_generic_dpad_location == GenericDpadNibbleLocation::Byte5LowNibble) {
            // Low nibble is the hat; avoid exposing it as buttons 1-4.
            buttons_byte0 &= 0xF0;
        }
        next_buttons |= buttons_byte0;
    }
    if (length > 6) next_buttons |= (uint32_t)data[6] << 8;

    // Heuristic remap for common "cheap USB gamepad" reports:
    // When the D-pad is encoded as a hat in the low nibble of byte 5, some
    // controllers put the four face buttons in the *high* nibble (bits 4-7).
    // If we forward those bits as-is, they end up on LB/RB/START/BACK in our
    // normalized layout and hosts see them as non-face BTN_* codes.
    //
    // If we see this pattern (buttons 1-4 unused, buttons 5-8 used), remap the
    // nibble into our standard A/B/X/Y bits using the observed order:
    // bit4=Y, bit5=B, bit6=A, bit7=X.
    if (g_generic_dpad_location == GenericDpadNibbleLocation::Byte5LowNibble) {
        const uint32_t low_nibble = next_buttons & 0x0Fu;
        const uint32_t face_bits = next_buttons & 0xF0u;
        const bool has_hat_and_face_in_high_nibble = (low_nibble == 0 && face_bits != 0);

        if (has_hat_and_face_in_high_nibble) {
            // Observed order for these controllers:
            // bit4=Y, bit5=B, bit6=A, bit7=X.
            next_buttons &= ~0xF0u;
            if (face_bits & 0x10) next_buttons |= (1u << GAMEPAD_BUTTON_Y);
            if (face_bits & 0x20) next_buttons |= (1u << GAMEPAD_BUTTON_B);
            if (face_bits & 0x40) next_buttons |= (1u << GAMEPAD_BUTTON_A);
            if (face_bits & 0x80) next_buttons |= (1u << GAMEPAD_BUTTON_X);

            // More observed mappings on the same class of controller:
            // raw bit11 = right shoulder, raw bit12 = start.
            const uint32_t raw_rb = (1u << 11);
            const uint32_t raw_start = (1u << 12);
            if (next_buttons & raw_rb) {
                next_buttons = (next_buttons & ~raw_rb) | (1u << GAMEPAD_BUTTON_RB);
            }
            if (next_buttons & raw_start) {
                next_buttons = (next_buttons & ~raw_start) | (1u << GAMEPAD_BUTTON_START);
            }
        }
    }

    const int16_t next_left_trigger = 0;
    const int16_t next_right_trigger = 0;

    if (xSemaphoreTake(g_gamepad_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    const GamepadState prev_state = g_gamepad_state;

    const bool state_changed =
        prev_state.left_stick_x != next_left_stick_x ||
        prev_state.left_stick_y != next_left_stick_y ||
        prev_state.right_stick_x != next_right_stick_x ||
        prev_state.right_stick_y != next_right_stick_y ||
        prev_state.left_trigger != next_left_trigger ||
        prev_state.right_trigger != next_right_trigger ||
        prev_state.dpad != next_dpad ||
        prev_state.buttons != next_buttons;

    if (state_changed) {
        const uint32_t pressed_mask = (~prev_state.buttons) & next_buttons;

        g_gamepad_state.left_stick_x = next_left_stick_x;
        g_gamepad_state.left_stick_y = next_left_stick_y;
        g_gamepad_state.right_stick_x = next_right_stick_x;
        g_gamepad_state.right_stick_y = next_right_stick_y;
        g_gamepad_state.left_trigger = next_left_trigger;
        g_gamepad_state.right_trigger = next_right_trigger;
        g_gamepad_state.dpad = next_dpad;
        g_gamepad_state.buttons = next_buttons;
        g_gamepad_state.changed = true;

        if (pressed_mask) {
            log_button_presses(pressed_mask);
        }
    }

    xSemaphoreGive(g_gamepad_mutex);
}

// Parse DualShock 4 (PlayStation 4) controller report.
// 64 bytes, report ID 0x01.
static void parse_dualshock4_report(const uint8_t* data, size_t length) {
    if (length < 12) {
        parse_generic_8byte_report(data, length);
        return;
    }

    const int16_t next_left_stick_x = normalize_stick_axis_u8(data[1]);
    const int16_t next_left_stick_y = normalize_stick_axis_u8(data[2]);
    const int16_t next_right_stick_x = normalize_stick_axis_u8(data[3]);
    const int16_t next_right_stick_y = normalize_stick_axis_u8(data[4]);
    const uint8_t next_dpad = dpad_value_to_hat_legacy(data[5] & 0x0F);

    if (xSemaphoreTake(g_gamepad_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    const GamepadState prev_state = g_gamepad_state;

    // DualShock 4 report structure:
    // [0]: Report ID (0x01)
    // [1]: Left stick X (0-255, 128 = center)
    // [2]: Left stick Y (0-255, 128 = center)
    // [3]: Right stick X
    // [4]: Right stick Y
    // [5]: D-pad (low 4 bits) + buttons (high 4 bits: triangle, circle, cross, square)
    // [6]: Buttons (R3, L3, Options, Share, R2, L2, R1, L1)
    // [7]: Buttons (touchpad click, logo) + report counter
    // [8]: L2 trigger analog
    // [9]: R2 trigger analog

    // Map DS4 buttons to standard layout.
    uint32_t next_buttons = 0;
    uint8_t btn1 = (data[5] >> 4) & 0x0F;
    uint8_t btn2 = data[6];

    if (btn1 & 0x02) next_buttons |= (1 << GAMEPAD_BUTTON_A);       // Cross
    if (btn1 & 0x04) next_buttons |= (1 << GAMEPAD_BUTTON_B);       // Circle
    if (btn1 & 0x01) next_buttons |= (1 << GAMEPAD_BUTTON_X);       // Square
    if (btn1 & 0x08) next_buttons |= (1 << GAMEPAD_BUTTON_Y);       // Triangle
    if (btn2 & 0x01) next_buttons |= (1 << GAMEPAD_BUTTON_LB);      // L1
    if (btn2 & 0x02) next_buttons |= (1 << GAMEPAD_BUTTON_RB);      // R1
    if (btn2 & 0x10) next_buttons |= (1 << GAMEPAD_BUTTON_BACK);    // Share
    if (btn2 & 0x20) next_buttons |= (1 << GAMEPAD_BUTTON_START);   // Options
    if (btn2 & 0x40) next_buttons |= (1 << GAMEPAD_BUTTON_L3);      // L3
    if (btn2 & 0x80) next_buttons |= (1 << GAMEPAD_BUTTON_R3);      // R3
    if (data[7] & 0x01) next_buttons |= (1 << GAMEPAD_BUTTON_GUIDE); // Logo

    const int16_t next_left_trigger = normalize_trigger_axis_u8(data[8]);
    const int16_t next_right_trigger = normalize_trigger_axis_u8(data[9]);

    const bool state_changed =
        prev_state.left_stick_x != next_left_stick_x ||
        prev_state.left_stick_y != next_left_stick_y ||
        prev_state.right_stick_x != next_right_stick_x ||
        prev_state.right_stick_y != next_right_stick_y ||
        prev_state.left_trigger != next_left_trigger ||
        prev_state.right_trigger != next_right_trigger ||
        prev_state.dpad != next_dpad ||
        prev_state.buttons != next_buttons;

    if (state_changed) {
        const uint32_t pressed_mask = (~prev_state.buttons) & next_buttons;

        g_gamepad_state.left_stick_x = next_left_stick_x;
        g_gamepad_state.left_stick_y = next_left_stick_y;
        g_gamepad_state.right_stick_x = next_right_stick_x;
        g_gamepad_state.right_stick_y = next_right_stick_y;
        g_gamepad_state.left_trigger = next_left_trigger;
        g_gamepad_state.right_trigger = next_right_trigger;
        g_gamepad_state.dpad = next_dpad;
        g_gamepad_state.buttons = next_buttons;
        g_gamepad_state.changed = true;

        if (pressed_mask) {
            log_button_presses(pressed_mask);
        }
    }

    xSemaphoreGive(g_gamepad_mutex);
}

void parse_hid_report(const uint8_t* data, size_t length) {
    // If we have a valid parsed HID descriptor, use it.
    if (g_hid_report_map_valid) {
        parse_descriptor_based_report(data, length);
        return;
    }

    // Log once that we're using fallback parsing.
    static bool logged_fallback = false;
    if (!logged_fallback) {
        ESP_LOGW(TAG, "No valid HID descriptor, using fallback heuristic parsing");
        logged_fallback = true;
    }

    // Fall back to heuristic parsing based on report length.
    switch (length) {
        case 64:
            // DualShock 4 / DualSense style report.
            if (data[0] == 0x01) {
                parse_dualshock4_report(data, length);
                return;
            }
            break;

        default:
            break;
    }

    // Fall back to generic parsing.
    parse_generic_8byte_report(data, length);
}

void hid_parser_reset() {
    g_generic_dpad_location = GenericDpadNibbleLocation::Unknown;
}
