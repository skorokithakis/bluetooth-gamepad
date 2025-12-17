#pragma once

#include <stdint.h>
#include <stddef.h>

#include "hid_descriptor.h"

// Global report map populated when a device connects.
// This is set by usb_host.cpp after parsing the HID descriptor.
extern HidReportMapCollection g_hid_report_map;
extern bool g_hid_report_map_valid;

// Parse a raw HID input report and update g_gamepad_state.
void parse_hid_report(const uint8_t* data, size_t length);

// Reset any per-device parsing heuristics (call on device connect/disconnect).
void hid_parser_reset();
