#include <Arduino.h>

#include "gamepad_state.h"
#include "usb_host.h"
#include "ble_gamepad.h"
#include "debug.h"

// Pin definitions for Seeed XIAO ESP32-S3.
static const int PIN_RGB_LED = 48;
static const int PIN_BOOT_BUTTON = 0;

// RGB LED is active-low on this board.
static const bool LED_ON = LOW;
static const bool LED_OFF = HIGH;

// Global gamepad state shared between USB input and BLE output.
GamepadState g_gamepad_state = {0};
SemaphoreHandle_t g_gamepad_mutex = nullptr;

// LED state tracking.
enum LedState {
    LED_BOOTING,
    LED_BLE_ADVERTISING,
    LED_BLE_CONNECTED,
    LED_FULLY_CONNECTED,
};

static LedState g_led_state = LED_BOOTING;
static unsigned long g_led_last_toggle = 0;
static bool g_led_blink_on = false;
static unsigned long g_activity_flash_until = 0;

// Button debouncing.
static unsigned long g_button_pressed_at = 0;
static bool g_button_was_pressed = false;

static void update_led() {
    unsigned long now = millis();

    // Activity flash overrides other states briefly.
    if (now < g_activity_flash_until) {
        digitalWrite(PIN_RGB_LED, LED_ON);
        return;
    }

    switch (g_led_state) {
        case LED_BOOTING:
            // Red blinking - but we only have one LED, so just blink.
            if (now - g_led_last_toggle > 200) {
                g_led_blink_on = !g_led_blink_on;
                g_led_last_toggle = now;
            }
            digitalWrite(PIN_RGB_LED, g_led_blink_on ? LED_ON : LED_OFF);
            break;

        case LED_BLE_ADVERTISING:
            // Slow blink while waiting for BLE connection.
            if (now - g_led_last_toggle > 500) {
                g_led_blink_on = !g_led_blink_on;
                g_led_last_toggle = now;
            }
            digitalWrite(PIN_RGB_LED, g_led_blink_on ? LED_ON : LED_OFF);
            break;

        case LED_BLE_CONNECTED:
            // Solid on, but no gamepad yet.
            digitalWrite(PIN_RGB_LED, LED_ON);
            break;

        case LED_FULLY_CONNECTED:
            // Solid on, both BLE and gamepad connected.
            digitalWrite(PIN_RGB_LED, LED_ON);
            break;
    }
}

static void update_led_state() {
    bool ble_connected = ble_gamepad_connected();
#ifndef DISABLE_USB_HOST
    bool gamepad_connected = usb_host_gamepad_connected();
#else
    bool gamepad_connected = false;
#endif

    if (!ble_connected) {
        g_led_state = LED_BLE_ADVERTISING;
    } else if (!gamepad_connected) {
        g_led_state = LED_BLE_CONNECTED;
    } else {
        g_led_state = LED_FULLY_CONNECTED;
    }
}

static void check_pairing_button() {
    bool pressed = (digitalRead(PIN_BOOT_BUTTON) == LOW);
    unsigned long now = millis();

    if (pressed && !g_button_was_pressed) {
        g_button_pressed_at = now;
        g_button_was_pressed = true;
    } else if (!pressed && g_button_was_pressed) {
        // Button released.
        unsigned long duration = now - g_button_pressed_at;
        g_button_was_pressed = false;

        // Require at least 50ms press to debounce.
        if (duration > 50 && duration < 3000) {
            ble_gamepad_clear_bonds();
        }
    }
}

void setup() {
    pinMode(PIN_RGB_LED, OUTPUT);
    digitalWrite(PIN_RGB_LED, LED_ON);

    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

    // Initialize debug serial first so we can see boot messages.
    debug_init();
    debug_log("Boot: Starting USB-BT Gamepad...");

    // Create mutex for thread-safe access to gamepad state.
    g_gamepad_mutex = xSemaphoreCreateMutex();
    debug_log("Boot: Mutex created");

    // Initialize BLE Gamepad first (starts advertising).
    debug_log("Boot: Initializing BLE...");
    ble_gamepad_init();

#ifndef DISABLE_USB_HOST
    // Optional delay before bringing up USB Host (helps keep boot quieter and gives
    // you a brief window before the device starts acting as a host).
#ifdef USB_HOST_START_DELAY_MS
    if (USB_HOST_START_DELAY_MS > 0) {
        debug_log("Boot: Delaying USB Host start by %d ms", (int)USB_HOST_START_DELAY_MS);
        delay(USB_HOST_START_DELAY_MS);
    }
#endif

    // Initialize USB Host.
    debug_log("Boot: Initializing USB Host...");
    if (!usb_host_init()) {
        debug_log("ERROR: USB Host init failed!");
        // USB Host failed - keep LED blinking rapidly to indicate error.
        while (true) {
            digitalWrite(PIN_RGB_LED, LED_ON);
            delay(100);
            digitalWrite(PIN_RGB_LED, LED_OFF);
            delay(100);
        }
    }
    debug_log("Boot: USB Host initialized");
#else
    debug_log("Boot: USB Host disabled for debugging");
#endif

    g_led_state = LED_BLE_ADVERTISING;
    debug_log("Boot: Ready, waiting for connections...");
}

void loop() {
    // Check for gamepad state updates from USB with mutex protection.
    GamepadState local_state;
    bool has_update = false;

    if (xSemaphoreTake(g_gamepad_mutex, 0) == pdTRUE) {
        if (g_gamepad_state.changed) {
            local_state = g_gamepad_state;
            g_gamepad_state.changed = false;
            has_update = true;
        }
        xSemaphoreGive(g_gamepad_mutex);
    }

    if (has_update) {
        ble_gamepad_send(local_state);
        g_activity_flash_until = millis() + 50;
    }

    // Update status.
    update_led_state();
    update_led();

    // Check pairing button.
    check_pairing_button();

    // Periodic debug status output.
    debug_print_status();

    // Small yield to prevent watchdog.
    delay(1);
}
