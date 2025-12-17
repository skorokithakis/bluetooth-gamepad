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

static const char* k_debug_eol = "\r\n";

const char* debug_eol() {
    return k_debug_eol;
}

static void debug_serial_print_eol() {
    DEBUG_SERIAL.print(debug_eol());
}

void debug_init() {
#ifdef DISABLE_USB_HOST
    DEBUG_SERIAL.begin(115200);
#else
    DEBUG_SERIAL.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
#endif

    // Wait briefly for serial to initialize.
    delay(100);

    debug_serial_print_eol();
    DEBUG_SERIAL.print("===========================================");
    debug_serial_print_eol();
    DEBUG_SERIAL.print("  USB-BT Gamepad Debug Console");
    debug_serial_print_eol();
    DEBUG_SERIAL.print("===========================================");
    debug_serial_print_eol();
    debug_serial_print_eol();
}

void debug_log(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    DEBUG_SERIAL.printf("[%8lu] %s%s", millis(), buffer, debug_eol());
}

void debug_printfln(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    DEBUG_SERIAL.print(buffer);
    debug_serial_print_eol();
}

void debug_print_gamepad_state(const GamepadState& state) {
    debug_printfln(
        "  Sticks: L(%5d,%5d) R(%5d,%5d)",
        state.left_stick_x, state.left_stick_y,
        state.right_stick_x, state.right_stick_y
    );
    debug_printfln(
        "  Triggers: L=%5d R=%5d  D-pad=%d",
        state.left_trigger, state.right_trigger, state.dpad
    );
    debug_printfln("  Buttons: 0x%04X", state.buttons);
}

void debug_print_status() {
    unsigned long now = millis();

    if (now - g_last_status_print < STATUS_PRINT_INTERVAL_MS) {
        return;
    }
    g_last_status_print = now;

    bool ble_connected = ble_gamepad_connected();

    DEBUG_SERIAL.print("--- Status ---");
    debug_serial_print_eol();
    DEBUG_SERIAL.printf("  BLE: %s%s", ble_connected ? "CONNECTED" : "advertising...", debug_eol());
#ifndef DISABLE_USB_HOST
    bool gamepad_connected = usb_host_gamepad_connected();
    DEBUG_SERIAL.printf("  USB Gamepad: %s%s", gamepad_connected ? "CONNECTED" : "not connected", debug_eol());
#else
    DEBUG_SERIAL.print("  USB Gamepad: disabled");
    debug_serial_print_eol();
#endif

    if (ble_connected) {
        ble_gamepad_print_connection_info();
    }

    debug_serial_print_eol();
}
