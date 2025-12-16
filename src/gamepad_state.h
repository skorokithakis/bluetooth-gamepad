#pragma once

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Normalized gamepad state shared between USB input and BLE output.
// All axes use the range 0-32767 to match BLE Gamepad defaults.
struct GamepadState {
    uint32_t buttons;

    int16_t left_stick_x;
    int16_t left_stick_y;
    int16_t right_stick_x;
    int16_t right_stick_y;

    int16_t left_trigger;
    int16_t right_trigger;

    // Hat switch position (0 = centered, 1-8 = directions clockwise from up).
    uint8_t dpad;

    // Set to true when state has been updated and needs to be sent over BLE.
    bool changed;
};

// Button bit positions matching common gamepad layouts.
enum GamepadButton {
    GAMEPAD_BUTTON_A = 0,
    GAMEPAD_BUTTON_B = 1,
    GAMEPAD_BUTTON_X = 2,
    GAMEPAD_BUTTON_Y = 3,
    GAMEPAD_BUTTON_LB = 4,
    GAMEPAD_BUTTON_RB = 5,
    GAMEPAD_BUTTON_BACK = 6,
    GAMEPAD_BUTTON_START = 7,
    GAMEPAD_BUTTON_L3 = 8,
    GAMEPAD_BUTTON_R3 = 9,
    GAMEPAD_BUTTON_GUIDE = 10,
};

// Hat switch directions matching BLE Gamepad HAT_* constants.
enum HatDirection {
    HAT_CENTERED = 0,
    HAT_UP = 1,
    HAT_UP_RIGHT = 2,
    HAT_RIGHT = 3,
    HAT_DOWN_RIGHT = 4,
    HAT_DOWN = 5,
    HAT_DOWN_LEFT = 6,
    HAT_LEFT = 7,
    HAT_UP_LEFT = 8,
};

extern GamepadState g_gamepad_state;
extern SemaphoreHandle_t g_gamepad_mutex;
