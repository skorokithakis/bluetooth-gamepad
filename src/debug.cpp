#include "debug.h"
#include "ble_gamepad.h"
#ifndef DISABLE_USB_HOST
#include "usb_host.h"
#endif

#include <Arduino.h>
#include <stdarg.h>

#ifdef DISABLE_USB_HOST
// USB CDC available for debug output.
#define DEBUG_SERIAL Serial
#else
// USB-OTG used for host mode, use hardware UART0 instead.
#define DEBUG_SERIAL Serial0
static const int DEBUG_TX_PIN = 12;
static const int DEBUG_RX_PIN = 13;
#endif

static unsigned long g_last_status_print = 0;
static const unsigned long STATUS_PRINT_INTERVAL_MS = 2000;

void debug_init() {
#ifdef DISABLE_USB_HOST
    DEBUG_SERIAL.begin(115200);
#else
    DEBUG_SERIAL.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
#endif

    // Wait briefly for serial to initialize.
    delay(100);

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("===========================================");
    DEBUG_SERIAL.println("  USB-BT Gamepad Debug Console");
    DEBUG_SERIAL.println("===========================================");
    DEBUG_SERIAL.println();
}

void debug_log(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    DEBUG_SERIAL.printf("[%8lu] %s\n", millis(), buffer);
}

void debug_print_gamepad_state(const GamepadState& state) {
    DEBUG_SERIAL.printf(
        "  Sticks: L(%5d,%5d) R(%5d,%5d)\n",
        state.left_stick_x, state.left_stick_y,
        state.right_stick_x, state.right_stick_y
    );
    DEBUG_SERIAL.printf(
        "  Triggers: L=%5d R=%5d  D-pad=%d\n",
        state.left_trigger, state.right_trigger, state.dpad
    );
    DEBUG_SERIAL.printf("  Buttons: 0x%04X\n", state.buttons);
}

void debug_print_status() {
    unsigned long now = millis();

    if (now - g_last_status_print < STATUS_PRINT_INTERVAL_MS) {
        return;
    }
    g_last_status_print = now;

    bool ble_connected = ble_gamepad_connected();

    DEBUG_SERIAL.println("--- Status ---");
    DEBUG_SERIAL.printf("  BLE: %s\n", ble_connected ? "CONNECTED" : "advertising...");
#ifndef DISABLE_USB_HOST
    bool gamepad_connected = usb_host_gamepad_connected();
    DEBUG_SERIAL.printf("  USB Gamepad: %s\n", gamepad_connected ? "CONNECTED" : "not connected");
#else
    DEBUG_SERIAL.println("  USB Gamepad: disabled");
#endif

    if (ble_connected) {
        ble_gamepad_print_connection_info();
    }

    DEBUG_SERIAL.println();
}
