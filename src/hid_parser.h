#pragma once

#include <stdint.h>
#include <stddef.h>

// Parse a raw HID input report and update g_gamepad_state.
void parse_hid_report(const uint8_t* data, size_t length);
