#include "hid_parser.h"
#include "gamepad_state.h"

#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "HID_PARSER";

static const char* button_name_for_bit(int bit) {
    switch (bit) {
        case GAMEPAD_BUTTON_A: return "A";
        case GAMEPAD_BUTTON_B: return "B";
        case GAMEPAD_BUTTON_X: return "X";
        case GAMEPAD_BUTTON_Y: return "Y";
        case GAMEPAD_BUTTON_LB: return "LB";
        case GAMEPAD_BUTTON_RB: return "RB";
        case GAMEPAD_BUTTON_BACK: return "BACK";
        case GAMEPAD_BUTTON_START: return "START";
        case GAMEPAD_BUTTON_L3: return "L3";
        case GAMEPAD_BUTTON_R3: return "R3";
        case GAMEPAD_BUTTON_GUIDE: return "GUIDE";
        default: return nullptr;
    }
}

static void log_button_presses(uint32_t pressed_mask) {
    while (pressed_mask) {
        int bit = __builtin_ctz(pressed_mask);
        pressed_mask &= ~(1u << bit);

        const char* name = button_name_for_bit(bit);
        if (name) {
            ESP_LOGI(TAG, "Button pressed: %s", name);
        } else {
            ESP_LOGI(TAG, "Button pressed: %d", bit);
        }
    }
}

// Normalize an unsigned 8-bit axis value (0-255) to 0-32767.
static int16_t normalize_axis_u8(uint8_t value) {
    return (int16_t)((uint32_t)value * 32767 / 255);
}

// Convert 8-way D-pad value (0-7, 8/15 = centered) to hat direction.
static uint8_t dpad_value_to_hat(uint8_t value) {
    if (value > 7) return HAT_CENTERED;
    // D-pad values: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW.
    // Hat values: 1=N, 2=NE, 3=E, 4=SE, 5=S, 6=SW, 7=W, 8=NW.
    return value + 1;
}

// Parse generic 8-byte HID gamepad report.
// Common format for cheap USB gamepads.
static void parse_generic_8byte_report(const uint8_t* data, size_t length) {
    if (length < 6) return;

    const int16_t next_left_stick_x = normalize_axis_u8(data[0]);
    const int16_t next_left_stick_y = normalize_axis_u8(data[1]);
    const int16_t next_right_stick_x = normalize_axis_u8(data[2]);
    const int16_t next_right_stick_y = normalize_axis_u8(data[3]);
    const uint8_t next_dpad = dpad_value_to_hat(data[4] & 0x0F);

    uint32_t next_buttons = 0;
    if (length > 5) next_buttons |= data[5];
    if (length > 6) next_buttons |= (uint32_t)data[6] << 8;

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

    const int16_t next_left_stick_x = normalize_axis_u8(data[1]);
    const int16_t next_left_stick_y = normalize_axis_u8(data[2]);
    const int16_t next_right_stick_x = normalize_axis_u8(data[3]);
    const int16_t next_right_stick_y = normalize_axis_u8(data[4]);
    const uint8_t next_dpad = dpad_value_to_hat(data[5] & 0x0F);

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

    const int16_t next_left_trigger = normalize_axis_u8(data[8]);
    const int16_t next_right_trigger = normalize_axis_u8(data[9]);

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
    // Detect controller type by report length.
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
