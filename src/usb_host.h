#pragma once

#include <stdbool.h>

// Initialize the USB Host subsystem and HID driver.
// Returns true on success.
bool usb_host_init();

// Periodic housekeeping (optional).
// When enabled, can automatically restart the USB Host stack if a gamepad fails
// to enumerate after boot.
void usb_host_poll();

// Returns true if a HID gamepad is currently connected.
bool usb_host_gamepad_connected();
