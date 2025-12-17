#pragma once

#include <stdint.h>
#include <stddef.h>

// Maximum number of input fields we track from a HID descriptor.
#define HID_MAX_FIELDS 64
// Maximum number of distinct report IDs we track per device.
#define HID_MAX_REPORTS 8

// HID Usage Pages relevant for gamepads.
enum HidUsagePage : uint16_t {
    HID_USAGE_PAGE_GENERIC_DESKTOP = 0x01,
    HID_USAGE_PAGE_SIMULATION = 0x02,
    HID_USAGE_PAGE_BUTTON = 0x09,
    HID_USAGE_PAGE_CONSUMER = 0x0C,
};

// Generic Desktop usages.
enum HidUsageGenericDesktop : uint16_t {
    HID_USAGE_X = 0x30,
    HID_USAGE_Y = 0x31,
    HID_USAGE_Z = 0x32,
    HID_USAGE_RX = 0x33,
    HID_USAGE_RY = 0x34,
    HID_USAGE_RZ = 0x35,
    HID_USAGE_HAT_SWITCH = 0x39,
    HID_USAGE_START = 0x3D,
    HID_USAGE_SELECT = 0x3E,
};

// Simulation usages (some controllers use these for triggers).
enum HidUsageSimulation : uint16_t {
    HID_USAGE_THROTTLE = 0xBB,
    HID_USAGE_BRAKE = 0xC5,
};

// A single input field parsed from the HID report descriptor.
struct HidField {
    uint16_t usage_page;
    uint16_t usage;
    uint16_t bit_offset;    // Bit position in the report (after any report ID byte).
    uint8_t bit_size;       // Bits per value.
    uint8_t count;          // Number of sequential values (for button arrays, this can be > 1).
    int32_t logical_min;
    int32_t logical_max;
    bool is_variable;       // True for variable fields, false for array fields.
};

// Parsed HID report descriptor for a single report ID (or the default report).
struct HidReportMap {
    uint8_t report_id;              // 0 if no report ID is used.
    uint8_t report_byte_length;     // Expected report length in bytes (excluding report ID).
    uint8_t field_count;
    HidField fields[HID_MAX_FIELDS];
};

// Parsed HID report descriptor for devices that use multiple report IDs.
struct HidReportMapCollection {
    uint8_t report_count;
    HidReportMap reports[HID_MAX_REPORTS];
};

// Parse a raw HID report descriptor and extract input field mappings.
// Returns true if parsing succeeded and produced a usable report map.
bool hid_parse_report_descriptor(const uint8_t* descriptor, size_t length, HidReportMapCollection* out_maps);

// Find a field in the report map by usage page and usage.
// Returns nullptr if not found.
const HidField* hid_find_field(const HidReportMap* map, uint16_t usage_page, uint16_t usage);

// Find the first button field (usage page = Button).
// Returns nullptr if not found.
const HidField* hid_find_button_field(const HidReportMap* map);

// Find a report map by report ID. Returns nullptr if not found.
const HidReportMap* hid_find_report_map(const HidReportMapCollection* maps, uint8_t report_id);

// Extract a field's value from a raw HID report.
// The report pointer should point past any report ID byte.
// Returns 0 if the field would read beyond report_length.
int32_t hid_extract_field_value(const uint8_t* report, size_t report_length, const HidField* field, uint8_t index);
