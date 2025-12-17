#pragma once

#include <stdint.h>
#include <stddef.h>

// Parse a raw HID input report and update g_gamepad_state.
void parse_hid_report(const uint8_t* data, size_t length);

// Reset any per-device parsing heuristics (call on device connect/disconnect).
void hid_parser_reset();
