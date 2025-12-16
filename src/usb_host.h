#pragma once

#include <stdbool.h>

// Initialize the USB Host subsystem and HID driver.
// Returns true on success.
bool usb_host_init();

// Returns true if a HID gamepad is currently connected.
bool usb_host_gamepad_connected();
