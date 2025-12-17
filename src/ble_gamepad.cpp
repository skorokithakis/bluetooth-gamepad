#include "ble_gamepad.h"
#include "debug.h"
#include "gamepad_state.h"

#include <BleGamepad.h>
#include <NimBLEDevice.h>

static BleGamepad bleGamepad("USB-BT Gamepad", "DIY", 100);

// The ESP32-BLE-Gamepad library defaults to unsigned 16-bit axis ranges
// (LOGICAL_MINIMUM=0, LOGICAL_MAXIMUM=32767). Our internal stick values are
// normalized to signed -32767..32767, so convert them to 0..32767 before
// calling setAxes(), otherwise negative values wrap/clamp and cause twitching
// and missing "left" movement on hosts (e.g. Linux evdev).
static int16_t stick_s16_to_u15(int16_t value) {
    int32_t clamped = value;
    if (clamped < -32767) clamped = -32767;
    if (clamped > 32767) clamped = 32767;
    return (int16_t)((clamped + 32767) / 2);
}

void ble_gamepad_init() {
    BleGamepadConfiguration config;

    config.setAutoReport(false);
    config.setButtonCount(16);
    config.setHatSwitchCount(1);
    // Keep Start/Select as regular buttons (Button 11/12 in the BLE descriptor).
    // Some hosts (notably Linux/BlueZ) do not consistently map Generic Desktop
    // Start/Select usages to BTN_START/BTN_SELECT for gamepads.
    config.setIncludeStart(false);
    config.setIncludeSelect(false);

    config.setIncludeXAxis(true);
    config.setIncludeYAxis(true);
    config.setIncludeZAxis(true);
    config.setIncludeRxAxis(true);
    config.setIncludeRyAxis(true);
    config.setIncludeRzAxis(true);

    // Z and Rz are used for triggers.
    config.setIncludeSlider1(false);
    config.setIncludeSlider2(false);

    bleGamepad.begin(&config);

    // NimBLEDevice::init() happens asynchronously inside the BleGamepad task.
    // Wait briefly for it to finish before calling any NimBLE APIs that require it.
    bool nimble_ready = false;
    for (int i = 0; i < 50; i++) {  // ~500ms max
        if (NimBLEDevice::isInitialized()) {
            nimble_ready = true;
            break;
        }
        delay(10);
    }

    if (nimble_ready) {
        // Configure pairing/security.
        // Some hosts (including modern Linux/BlueZ setups) will disconnect with auth failures
        // if the peripheral does not support LE Secure Connections during pairing.
        NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // "Just Works"
        NimBLEDevice::setSecurityAuth(true, false, true);          // bonding, no MITM, secure connections
        NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
        NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

        debug_log("BLE: Gamepad initialized");
        debug_log("  Device name: USB-BT Gamepad");
        debug_log("  Address: %s", NimBLEDevice::getAddress().toString().c_str());

        // Print existing bonds.
        int bond_count = NimBLEDevice::getNumBonds();
        debug_log("  Bonded devices: %d", bond_count);
        for (int i = 0; i < bond_count; i++) {
            NimBLEAddress addr = NimBLEDevice::getBondedAddress(i);
            debug_log("    [%d] %s", i, addr.toString().c_str());
        }

        debug_log("BLE: Advertising started");
    } else {
        debug_log("BLE: NimBLE stack did not finish initializing in time");
    }
}

bool ble_gamepad_connected() {
    return bleGamepad.isConnected();
}

static uint8_t canonical_button_to_ble_button(GamepadButton button) {
    // Linux's HID gamepad mapping expects a "standard" button ordering.
    // Note: BLE button indices are 1-based.
    switch (button) {
        case GAMEPAD_BUTTON_A: return 1;       // BTN_SOUTH / BTN_A
        case GAMEPAD_BUTTON_B: return 2;       // BTN_EAST  / BTN_B
        case GAMEPAD_BUTTON_X: return 4;       // BTN_WEST  / BTN_X (3=BTN_C unused)
        case GAMEPAD_BUTTON_Y: return 5;       // BTN_NORTH / BTN_Y
        case GAMEPAD_BUTTON_LB: return 7;      // BTN_TL
        case GAMEPAD_BUTTON_RB: return 8;      // BTN_TR
        case GAMEPAD_BUTTON_BACK: return 11;   // BTN_SELECT
        case GAMEPAD_BUTTON_START: return 12;  // BTN_START
        case GAMEPAD_BUTTON_GUIDE: return 13;  // BTN_MODE / Guide
        case GAMEPAD_BUTTON_L3: return 14;     // BTN_THUMBL
        case GAMEPAD_BUTTON_R3: return 15;     // BTN_THUMBR
        default: return 0;
    }
}

void ble_gamepad_send(const GamepadState& state) {
    if (!bleGamepad.isConnected()) {
        return;
    }

    // Update buttons (canonical layout in GamepadState).
    for (uint8_t bit = 0; bit <= GAMEPAD_BUTTON_GUIDE; bit++) {
        const auto button = static_cast<GamepadButton>(bit);
        const uint8_t ble_button = canonical_button_to_ble_button(button);
        if (ble_button == 0) continue;
        const bool pressed = (state.buttons & (1u << bit)) != 0;
        if (pressed) bleGamepad.press(ble_button);
        else bleGamepad.release(ble_button);
    }

    // Update axes.
    // BLE Gamepad setAxes order: X, Y, Z, Rx, Ry, Rz, slider1, slider2.
    // We map: left stick X/Y, left trigger (Z), right stick X/Y, right trigger (Rz).
    bleGamepad.setAxes(
        stick_s16_to_u15(state.left_stick_x),
        stick_s16_to_u15(state.left_stick_y),
        state.left_trigger,
        stick_s16_to_u15(state.right_stick_x),
        stick_s16_to_u15(state.right_stick_y),
        state.right_trigger,
        0,
        0
    );

    // Update D-pad.
    bleGamepad.setHat1(state.dpad);

    bleGamepad.sendReport();
}

void ble_gamepad_clear_bonds() {
    debug_log("BLE: Clearing all bonds...");

    int bond_count = NimBLEDevice::getNumBonds();
    debug_log("  Removing %d bonded device(s)", bond_count);

    bleGamepad.end();
    NimBLEDevice::deleteAllBonds();

    debug_log("BLE: Reinitializing...");
    ble_gamepad_init();
}

void ble_gamepad_print_connection_info() {
    NimBLEServer* server = NimBLEDevice::getServer();
    if (!server) {
        debug_log("BLE: No server instance");
        return;
    }

    uint8_t conn_count = server->getConnectedCount();
    debug_log("  BLE connections: %d", conn_count);

    if (conn_count > 0) {
        // Get first connected client info by index.
        NimBLEConnInfo conn_info = server->getPeerInfo(0);
        debug_log("  Peer: %s", conn_info.getAddress().toString().c_str());
        debug_log("  Encrypted: %s, Bonded: %s",
            conn_info.isEncrypted() ? "yes" : "no",
            conn_info.isBonded() ? "yes" : "no");
    }
}
