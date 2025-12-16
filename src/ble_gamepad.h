#pragma once

#include "gamepad_state.h"

// Initialize the BLE Gamepad and start advertising.
void ble_gamepad_init();

// Returns true if a BLE host is connected.
bool ble_gamepad_connected();

// Send the current gamepad state over BLE.
void ble_gamepad_send(const GamepadState& state);

// Clear bonding information and restart advertising.
// Call this when the user presses the pairing button.
void ble_gamepad_clear_bonds();

// Print BLE connection info to debug serial.
void ble_gamepad_print_connection_info();
