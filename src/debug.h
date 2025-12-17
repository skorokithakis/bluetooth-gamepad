#pragma once

#include "gamepad_state.h"

// Debug end-of-line sequence. Use this instead of hardcoding '\n'/'\r\n'.
const char* debug_eol();

// Initialize hardware UART for debug output.
// Uses UART0 on GPIO12 (TX) / GPIO13 (RX) at 115200 baud.
void debug_init();

// Print current status (BLE state, gamepad connection, etc).
void debug_print_status();

// Print gamepad state details.
void debug_print_gamepad_state(const GamepadState& state);

// Print a message with timestamp.
void debug_log(const char* format, ...);

// Printf that automatically appends debug_eol().
void debug_printfln(const char* format, ...);
