//
// ups_battery.h - Windows native battery display via a HID UPS Battery interface.
//
// Windows has no battery icon for USB HID gamepads - the DualSense's charge
// state is only visible over Bluetooth. To show a native battery in the tray /
// power settings on Windows, the dongle can expose an EXTRA USB HID interface
// with the Power Device usage page (0x84) / Usage UPS (0x04). Windows' built-in
// hidups.sys / HidBatt stack binds to that interface and presents it as a
// system battery ("HID UPS Battery").
//
// The report layout mirrors forderud/HidBattery (MIT), a project verified to
// show battery percent on Windows 2000 through 11:
//   - PresentStatus + RemainingCapacity/FullChargeCapacity are the pair Windows
//     uses to compute the percentage.
//   - Feature reports are served via GET_REPORT at enumeration; input reports
//     are pushed every second from ups_battery_tick().
//
// Configuration (config.h / portal):
//   battery_mode 0 = off (no UPS interface), 1 = real controller battery,
//                2 = simulated fixed level (battery_fake).
//   battery_volt_blend: in Real mode, blend the coarse BT level (0-10) with
//     the factory-test battery voltage (mV) for a finer estimate.
//   battery_smooth: ease the reported percentage toward the target.
// Toggling 0 <-> non-zero changes the USB configuration descriptor, so the
// host must re-enumerate (portal ENUM_FIELDS / slot_activate needs_reenum).

#pragma once

#include <cstdint>

// Length of the UPS HID report descriptor (bytes). Kept in the header so
// usb_descriptors.cpp can embed it in the configuration descriptor's HID class
// descriptor; the static_assert in ups_battery.cpp guards the actual array.
constexpr uint16_t UPS_REPORT_DESC_LEN = 240;

// TinyUSB HID instance of the UPS interface, derived from which HID interfaces
// are present: the gamepad is always instance 0, the wake keyboard (when
// enabled) is instance 1, so the UPS is 1 or 2.
uint8_t ups_hid_instance();

// Pointer to the UPS HID report descriptor.
uint8_t const *ups_report_descriptor();

// Serve a GET_REPORT control request on the UPS interface (feature or input).
// Returns bytes written, 0 stalls the request.
uint16_t ups_get_report(uint8_t report_id, uint8_t report_type, uint8_t *buffer, uint16_t reqlen);

// Called from the main loop; rate-limits itself to 1 Hz and pushes the UPS
// input reports while battery_mode != 0. No-op when the feature is off.
void ups_battery_tick();

// Polls the controller for its factory-test battery voltage (mV) over the
// Bluetooth HID control channel. Best effort: returns 0 when the controller
// does not answer (retail controllers may reject the factory test command).
// Defined in ups_battery.cpp; uses bt.cpp's set/get feature report helpers.
uint16_t ups_read_battery_voltage_mv();
