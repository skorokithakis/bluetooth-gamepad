#include "hid_descriptor.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "HID_DESC";

// HID item types (bits 2-3 of the item prefix byte).
enum HidItemType : uint8_t {
    HID_ITEM_TYPE_MAIN = 0,
    HID_ITEM_TYPE_GLOBAL = 1,
    HID_ITEM_TYPE_LOCAL = 2,
};

// HID main item tags (bits 4-7 when type = Main).
enum HidMainTag : uint8_t {
    HID_MAIN_INPUT = 0x08,
    HID_MAIN_OUTPUT = 0x09,
    HID_MAIN_FEATURE = 0x0B,
    HID_MAIN_COLLECTION = 0x0A,
    HID_MAIN_END_COLLECTION = 0x0C,
};

// HID global item tags (bits 4-7 when type = Global).
enum HidGlobalTag : uint8_t {
    HID_GLOBAL_USAGE_PAGE = 0x00,
    HID_GLOBAL_LOGICAL_MIN = 0x01,
    HID_GLOBAL_LOGICAL_MAX = 0x02,
    HID_GLOBAL_PHYSICAL_MIN = 0x03,
    HID_GLOBAL_PHYSICAL_MAX = 0x04,
    HID_GLOBAL_UNIT_EXPONENT = 0x05,
    HID_GLOBAL_UNIT = 0x06,
    HID_GLOBAL_REPORT_SIZE = 0x07,
    HID_GLOBAL_REPORT_ID = 0x08,
    HID_GLOBAL_REPORT_COUNT = 0x09,
    HID_GLOBAL_PUSH = 0x0A,
    HID_GLOBAL_POP = 0x0B,
};

// HID local item tags (bits 4-7 when type = Local).
enum HidLocalTag : uint8_t {
    HID_LOCAL_USAGE = 0x00,
    HID_LOCAL_USAGE_MIN = 0x01,
    HID_LOCAL_USAGE_MAX = 0x02,
    HID_LOCAL_DESIGNATOR_INDEX = 0x03,
    HID_LOCAL_DESIGNATOR_MIN = 0x04,
    HID_LOCAL_DESIGNATOR_MAX = 0x05,
    HID_LOCAL_STRING_INDEX = 0x07,
    HID_LOCAL_STRING_MIN = 0x08,
    HID_LOCAL_STRING_MAX = 0x09,
    HID_LOCAL_DELIMITER = 0x0A,
};

// Input/Output/Feature item flags (data byte).
enum HidDataFlags : uint8_t {
    HID_DATA_CONSTANT = 0x01,       // Bit 0: 0 = Data, 1 = Constant.
    HID_DATA_VARIABLE = 0x02,       // Bit 1: 0 = Array, 1 = Variable.
    HID_DATA_RELATIVE = 0x04,       // Bit 2: 0 = Absolute, 1 = Relative.
};

// Parser state for global items.
struct HidGlobalState {
    uint16_t usage_page;
    int32_t logical_min;
    int32_t logical_max;
    uint8_t report_size;
    uint8_t report_count;
    uint8_t report_id;
};

// Parser state for local items (reset after each Main item).
struct HidLocalState {
    // We track multiple usages because a single Input item can span multiple fields,
    // each with a different usage.
    uint16_t usages[HID_MAX_FIELDS];
    uint8_t usage_count;
    uint16_t usage_min;
    uint16_t usage_max;
    bool has_usage_range;
};

// Read a signed integer from descriptor data, handling sign extension.
static int32_t read_signed(const uint8_t* data, uint8_t size) {
    int32_t value = 0;
    for (uint8_t i = 0; i < size; i++) {
        value |= (uint32_t)data[i] << (i * 8);
    }
    // Sign extend if the high bit is set.
    if (size > 0 && size < 4) {
        uint32_t sign_bit = 1u << (size * 8 - 1);
        if (value & sign_bit) {
            uint32_t mask = ~((1u << (size * 8)) - 1);
            value |= mask;
        }
    }
    return value;
}

// Read an unsigned integer from descriptor data.
static uint32_t read_unsigned(const uint8_t* data, uint8_t size) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < size; i++) {
        value |= (uint32_t)data[i] << (i * 8);
    }
    return value;
}

static void reset_local_state(HidLocalState* local) {
    local->usage_count = 0;
    local->usage_min = 0;
    local->usage_max = 0;
    local->has_usage_range = false;
}

static HidReportMap* get_or_create_report(HidReportMapCollection* maps, uint8_t report_id) {
    for (uint8_t i = 0; i < maps->report_count; i++) {
        if (maps->reports[i].report_id == report_id) return &maps->reports[i];
    }
    if (maps->report_count >= HID_MAX_REPORTS) {
        return nullptr;
    }
    HidReportMap* map = &maps->reports[maps->report_count++];
    memset(map, 0, sizeof(*map));
    map->report_id = report_id;
    return map;
}

bool hid_parse_report_descriptor(const uint8_t* descriptor, size_t length, HidReportMapCollection* out_maps) {
    if (!descriptor || !out_maps || length == 0) {
        return false;
    }

    memset(out_maps, 0, sizeof(HidReportMapCollection));

    HidGlobalState global = {};
    HidLocalState local = {};

    HidGlobalState global_stack[4] = {};
    uint8_t global_stack_depth = 0;

    uint16_t report_bit_offsets[HID_MAX_REPORTS] = {0};
    uint8_t current_report_id = 0;
    HidReportMap* current_map = get_or_create_report(out_maps, current_report_id);
    if (!current_map) {
        return false;
    }
    size_t pos = 0;

    while (pos < length) {
        uint8_t prefix = descriptor[pos];

        // Long items have prefix 0xFE - we skip them as they're rarely used.
        if (prefix == 0xFE) {
            if (pos + 2 >= length) break;
            uint8_t data_size = descriptor[pos + 1];
            pos += 3 + data_size;
            continue;
        }

        // Short item format: bits 0-1 = size, bits 2-3 = type, bits 4-7 = tag.
        uint8_t size_code = prefix & 0x03;
        uint8_t item_type = (prefix >> 2) & 0x03;
        uint8_t item_tag = (prefix >> 4) & 0x0F;

        // Size: 0 = 0 bytes, 1 = 1 byte, 2 = 2 bytes, 3 = 4 bytes.
        uint8_t data_size = (size_code == 3) ? 4 : size_code;

        if (pos + 1 + data_size > length) {
            ESP_LOGW(TAG, "Descriptor truncated at pos %u", (unsigned)pos);
            break;
        }

        const uint8_t* data = &descriptor[pos + 1];
        pos += 1 + data_size;

        switch (item_type) {
            case HID_ITEM_TYPE_GLOBAL:
                switch (item_tag) {
                    case HID_GLOBAL_USAGE_PAGE:
                        global.usage_page = (uint16_t)read_unsigned(data, data_size);
                        break;
                    case HID_GLOBAL_LOGICAL_MIN:
                        global.logical_min = read_signed(data, data_size);
                        break;
                    case HID_GLOBAL_LOGICAL_MAX:
                        global.logical_max = read_signed(data, data_size);
                        break;
                    case HID_GLOBAL_REPORT_SIZE:
                        global.report_size = (uint8_t)read_unsigned(data, data_size);
                        break;
                    case HID_GLOBAL_REPORT_COUNT:
                        global.report_count = (uint8_t)read_unsigned(data, data_size);
                        break;
                    case HID_GLOBAL_REPORT_ID:
                        global.report_id = (uint8_t)read_unsigned(data, data_size);
                        current_report_id = global.report_id;
                        current_map = get_or_create_report(out_maps, current_report_id);
                        break;
                    case HID_GLOBAL_PUSH:
                        if (global_stack_depth < (uint8_t)(sizeof(global_stack) / sizeof(global_stack[0]))) {
                            global_stack[global_stack_depth++] = global;
                        }
                        break;
                    case HID_GLOBAL_POP:
                        if (global_stack_depth > 0) {
                            global = global_stack[--global_stack_depth];
                            current_report_id = global.report_id;
                            current_map = get_or_create_report(out_maps, current_report_id);
                        }
                        break;
                }
                break;

            case HID_ITEM_TYPE_LOCAL:
                switch (item_tag) {
                    case HID_LOCAL_USAGE:
                        if (local.usage_count < HID_MAX_FIELDS) {
                            local.usages[local.usage_count++] = (uint16_t)read_unsigned(data, data_size);
                        }
                        break;
                    case HID_LOCAL_USAGE_MIN:
                        local.usage_min = (uint16_t)read_unsigned(data, data_size);
                        local.has_usage_range = true;
                        break;
                    case HID_LOCAL_USAGE_MAX:
                        local.usage_max = (uint16_t)read_unsigned(data, data_size);
                        local.has_usage_range = true;
                        break;
                }
                break;

            case HID_ITEM_TYPE_MAIN:
                switch (item_tag) {
                    case HID_MAIN_INPUT: {
                        if (!current_map) {
                            reset_local_state(&local);
                            break;
                        }

                        uint8_t flags = data_size > 0 ? data[0] : 0;
                        bool is_constant = (flags & HID_DATA_CONSTANT) != 0;
                        bool is_variable = (flags & HID_DATA_VARIABLE) != 0;

                        // Track bit offset per report ID.
                        uint8_t report_index = 0xFF;
                        for (uint8_t i = 0; i < out_maps->report_count; i++) {
                            if (out_maps->reports[i].report_id == current_report_id) {
                                report_index = i;
                                break;
                            }
                        }
                        if (report_index == 0xFF) {
                            reset_local_state(&local);
                            break;
                        }
                        uint16_t& bit_offset = report_bit_offsets[report_index];

                        // Skip constant (padding) fields but still advance the bit offset.
                        if (is_constant) {
                            bit_offset += global.report_size * global.report_count;
                            reset_local_state(&local);
                            break;
                        }

                        // For variable fields, each report_count item gets its own field entry.
                        // For array fields, we create one field entry covering all items.
                        if (is_variable) {
                            for (uint8_t i = 0; i < global.report_count; i++) {
                                if (current_map->field_count >= HID_MAX_FIELDS) break;

                                HidField* field = &current_map->fields[current_map->field_count++];
                                field->usage_page = global.usage_page;
                                field->bit_offset = bit_offset;
                                field->bit_size = global.report_size;
                                field->count = 1;
                                field->logical_min = global.logical_min;
                                field->logical_max = global.logical_max;
                                field->is_variable = true;

                                // Assign usage from local state.
                                if (local.has_usage_range) {
                                    field->usage = local.usage_min + i;
                                    if (field->usage > local.usage_max) {
                                        field->usage = local.usage_max;
                                    }
                                } else if (i < local.usage_count) {
                                    field->usage = local.usages[i];
                                } else if (local.usage_count > 0) {
                                    // Reuse last usage if not enough usages specified.
                                    field->usage = local.usages[local.usage_count - 1];
                                }

                                bit_offset += global.report_size;
                            }
                        } else {
                            // Array field: one entry covering all items.
                            // Common for buttons where the array contains pressed button indices.
                            if (current_map->field_count < HID_MAX_FIELDS) {
                                HidField* field = &current_map->fields[current_map->field_count++];
                                field->usage_page = global.usage_page;
                                field->bit_offset = bit_offset;
                                field->bit_size = global.report_size;
                                field->count = global.report_count;
                                field->logical_min = global.logical_min;
                                field->logical_max = global.logical_max;
                                field->is_variable = false;

                                // For arrays, usage_min/max define the range of possible values.
                                if (local.has_usage_range) {
                                    field->usage = local.usage_min;
                                } else if (local.usage_count > 0) {
                                    field->usage = local.usages[0];
                                }
                            }
                            bit_offset += global.report_size * global.report_count;
                        }

                        reset_local_state(&local);
                        break;
                    }

                    case HID_MAIN_OUTPUT:
                    case HID_MAIN_FEATURE:
                        // We only care about Input items for reading gamepad state.
                        // Still need to reset local state.
                        reset_local_state(&local);
                        break;

                    case HID_MAIN_COLLECTION:
                    case HID_MAIN_END_COLLECTION:
                        // Collections don't affect bit layout.
                        break;
                }
                break;
        }
    }

    // Calculate report byte length per report ID from total bits, and log.
    ESP_LOGI(TAG, "Parsed descriptor: %u report(s)", out_maps->report_count);
    for (uint8_t r = 0; r < out_maps->report_count; r++) {
        HidReportMap* map = &out_maps->reports[r];
        map->report_byte_length = (report_bit_offsets[r] + 7) / 8;

        ESP_LOGI(TAG, "  Report ID %u: %u fields, %u bytes",
                 map->report_id, map->field_count, map->report_byte_length);

        for (uint8_t i = 0; i < map->field_count; i++) {
            const HidField* f = &map->fields[i];
            const char* usage_name = "";
                if (f->usage_page == HID_USAGE_PAGE_GENERIC_DESKTOP) {
                    switch (f->usage) {
                        case HID_USAGE_X: usage_name = "X"; break;
                        case HID_USAGE_Y: usage_name = "Y"; break;
                        case HID_USAGE_Z: usage_name = "Z"; break;
                        case HID_USAGE_RX: usage_name = "Rx"; break;
                        case HID_USAGE_RY: usage_name = "Ry"; break;
                        case HID_USAGE_RZ: usage_name = "Rz"; break;
                        case HID_USAGE_HAT_SWITCH: usage_name = "Hat"; break;
                        case HID_USAGE_START: usage_name = "Start"; break;
                        case HID_USAGE_SELECT: usage_name = "Select"; break;
                    }
                } else if (f->usage_page == HID_USAGE_PAGE_BUTTON) {
                    usage_name = "Btn";
                }
            ESP_LOGI(TAG, "    Field %u: page=0x%02X usage=0x%02X (%s) offset=%u size=%u count=%u min=%ld max=%ld var=%d",
                     i, f->usage_page, f->usage, usage_name, f->bit_offset, f->bit_size, f->count,
                     (long)f->logical_min, (long)f->logical_max, f->is_variable);
        }
    }

    for (uint8_t r = 0; r < out_maps->report_count; r++) {
        if (out_maps->reports[r].field_count > 0) return true;
    }
    return false;
}

const HidField* hid_find_field(const HidReportMap* map, uint16_t usage_page, uint16_t usage) {
    if (!map) return nullptr;

    for (uint8_t i = 0; i < map->field_count; i++) {
        const HidField* f = &map->fields[i];
        if (f->usage_page == usage_page && f->usage == usage) {
            return f;
        }
    }
    return nullptr;
}

const HidField* hid_find_button_field(const HidReportMap* map) {
    if (!map) return nullptr;

    for (uint8_t i = 0; i < map->field_count; i++) {
        const HidField* f = &map->fields[i];
        if (f->usage_page == HID_USAGE_PAGE_BUTTON) {
            return f;
        }
    }
    return nullptr;
}

const HidReportMap* hid_find_report_map(const HidReportMapCollection* maps, uint8_t report_id) {
    if (!maps) return nullptr;
    for (uint8_t i = 0; i < maps->report_count; i++) {
        if (maps->reports[i].report_id == report_id) return &maps->reports[i];
    }
    return nullptr;
}

int32_t hid_extract_field_value(const uint8_t* report, size_t report_length, const HidField* field, uint8_t index) {
    if (!report || !field || report_length == 0) return 0;

    // Use 32-bit arithmetic to avoid overflow with large index values.
    uint32_t bit_pos = (uint32_t)field->bit_offset + ((uint32_t)index * field->bit_size);
    uint32_t byte_pos = bit_pos / 8;
    uint8_t bit_in_byte = bit_pos % 8;

    // Calculate the last byte we need to read.
    uint32_t last_bit = bit_pos + field->bit_size - 1;
    uint32_t last_byte = last_bit / 8;
    if (last_byte >= report_length) {
        return 0;
    }

    // Extract value spanning up to 4 bytes.
    uint32_t raw = 0;
    uint8_t bits_remaining = field->bit_size;
    uint8_t bits_read = 0;

    while (bits_remaining > 0 && byte_pos < report_length) {
        uint8_t bits_in_this_byte = 8 - bit_in_byte;
        if (bits_in_this_byte > bits_remaining) {
            bits_in_this_byte = bits_remaining;
        }

        uint8_t mask = ((1u << bits_in_this_byte) - 1) << bit_in_byte;
        uint8_t byte_val = (report[byte_pos] & mask) >> bit_in_byte;
        raw |= (uint32_t)byte_val << bits_read;

        bits_read += bits_in_this_byte;
        bits_remaining -= bits_in_this_byte;
        byte_pos++;
        bit_in_byte = 0;
    }

    // Sign extend if logical_min is negative.
    if (field->logical_min < 0 && field->bit_size < 32) {
        uint32_t sign_bit = 1u << (field->bit_size - 1);
        if (raw & sign_bit) {
            raw |= ~((1u << field->bit_size) - 1);
        }
    }

    return (int32_t)raw;
}
