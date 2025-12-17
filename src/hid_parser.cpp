#include "hid_parser.h"
#include "gamepad_state.h"

#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "HID_PARSER";

#ifndef STICK_DEADZONE
// Deadzone applied to normalized stick axes (-32767..32767).
// Override via PlatformIO build flag `-DSTICK_DEADZONE=<value>`.
#define STICK_DEADZONE 3000
#endif

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
static uint8_t dpad_value_to_hat(uint8_t value) {
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
    const uint8_t next_dpad = dpad_value_to_hat(dpad_raw);

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
        if (low_nibble == 0 && face_bits != 0) {
            next_buttons &= ~0xF0u;
            if (face_bits & 0x10) next_buttons |= (1u << GAMEPAD_BUTTON_Y);
            if (face_bits & 0x20) next_buttons |= (1u << GAMEPAD_BUTTON_B);
            if (face_bits & 0x40) next_buttons |= (1u << GAMEPAD_BUTTON_A);
            if (face_bits & 0x80) next_buttons |= (1u << GAMEPAD_BUTTON_X);
        }

        // Additional compatibility for minimal controllers without stick clicks:
        // Some devices report START/SELECT as higher-numbered buttons that land
        // in our L3 and GUIDE bits when treated as a flat bitmap. If we detect
        // those bits but not the expected START/BACK bits, remap them.
        const uint32_t start_mask = (1u << GAMEPAD_BUTTON_START);
        const uint32_t back_mask = (1u << GAMEPAD_BUTTON_BACK);
        const uint32_t l3_mask = (1u << GAMEPAD_BUTTON_L3);
        const uint32_t guide_mask = (1u << GAMEPAD_BUTTON_GUIDE);

        if ((next_buttons & l3_mask) && !(next_buttons & start_mask)) {
            next_buttons = (next_buttons & ~l3_mask) | start_mask;
        }
        if ((next_buttons & guide_mask) && !(next_buttons & back_mask)) {
            next_buttons = (next_buttons & ~guide_mask) | back_mask;
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

void hid_parser_reset() {
    g_generic_dpad_location = GenericDpadNibbleLocation::Unknown;
}
