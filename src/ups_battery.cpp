//
// ups_battery.cpp - HID UPS Battery interface implementation.
//

#include "ups_battery.h"

#include <cstdint>

#include "tusb.h"
#include "config.h"
#include "macro.h"
#include "pico/time.h"

extern uint8_t interrupt_in_data[63]; // DualSense payload: [52] battery, [53] headset/mic

// PresentStatus bit positions as mapped by Windows' hidups.sys / HidBatt stack
// (same bit order as forderud/HidBattery). Bit 3 maps to BATTERY_CRITICAL.
static constexpr uint8_t UPS_ST_CHARGING        = 0x01;
static constexpr uint8_t UPS_ST_DISCHARGING     = 0x02;
static constexpr uint8_t UPS_ST_AC_PRESENT      = 0x04;
static constexpr uint8_t UPS_ST_SHUTDOWN_IMMIN  = 0x08;

// Capacities in AmpSec (the descriptor declares this unit). Windows only uses
// the RemainingCapacity / FullChargeCapacity RATIO for the tray percentage, so
// 100/100 simply means "0-100%".
static constexpr uint16_t UPS_FULL_CAPACITY   = 100;
static constexpr uint16_t UPS_DESIGN_CAPACITY = 100;
static constexpr uint16_t UPS_CRITICAL_LIMIT  = 5;   // RemainingCapacityLimit  (DefaultAlert1)
static constexpr uint16_t UPS_WARN_LIMIT      = 10;  // WarningCapacityLimit    (DefaultAlert2)

// Never report a level below this: Windows treats a critical HID UPS battery as
// a system battery and may trigger its configured critical action (hibernate /
// shutdown) on the emulated device. 5% still lets users simulate "low battery"
// without tripping that.
static constexpr uint8_t  UPS_SAFETY_FLOOR    = 5;

// Packed manufacturer date (2026-08-14): (year-1980)*512 + month*32 + day.
static constexpr uint16_t UPS_MANUFACTURE_DATE = (2026u - 1980u) * 512u + 8u * 32u + 14u;

// Report IDs used by the UPS interface.
enum {
    UPS_RID_IPRODUCT        = 0x01,
    UPS_RID_ISERIAL         = 0x02,
    UPS_RID_IMANUFACTURER   = 0x03,
    UPS_RID_IDEVICECHEMISTRY= 0x04,
    UPS_RID_PRESENTSTATUS   = 0x07,
    UPS_RID_MANUFACTUREDATE = 0x09,
    UPS_RID_TEMPERATURE     = 0x0A,
    UPS_RID_VOLTAGE         = 0x0B,
    UPS_RID_REMAINING       = 0x0C,
    UPS_RID_RUNTIMETOEMPTY  = 0x0D,
    UPS_RID_FULLCHARGE      = 0x0E,
    UPS_RID_DESIGNCAPACITY  = 0x0F,
    UPS_RID_REMNCAPLIMIT    = 0x10,
    UPS_RID_WARNCAPLIMIT    = 0x11,
    UPS_RID_CYCLECOUNT      = 0x14,
    UPS_RID_CAPACITYMODE    = 0x16,
};

// HID report descriptor for a UPS battery, modeled on forderud/HidBattery.
// Windows needs PresentStatus + RemainingCapacity + FullChargeCapacity to show
// a percentage; the other items are the ones hidups.sys reads at enumeration.
uint8_t const ups_report_descriptor_bytes[] = {
    0x05, 0x84,       // Usage Page (Power Device)
    0x09, 0x04,       // Usage (UPS)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x24,       //   Usage (PowerSummary)
    0xA1, 0x02,       //   Collection (Logical)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x15, 0x00,       //     Logical Minimum (0)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    // iProduct string index (feature)
    0x85, 0x01,       //     Report ID (1)
    0x09, 0xFE,       //     Usage (iProduct)
    0xB1, 0x23,       //     Feature (Const, Var, Abs, No Wrap, Linear, No Preferred, No Null, Non-volatile, Bitfield)
    // iSerialNumber string index (feature)
    0x85, 0x02,       //     Report ID (2)
    0x09, 0xFF,       //     Usage (iSerialNumber)
    0xB1, 0x23,       //     Feature
    // iManufacturer string index (feature)
    0x85, 0x03,       //     Report ID (3)
    0x09, 0xFD,       //     Usage (iManufacturer)
    0xB1, 0x23,       //     Feature
    0x05, 0x85,       //     Usage Page (Battery System)
    // iDeviceChemistry string index (feature)
    0x85, 0x04,       //     Report ID (4)
    0x09, 0x89,       //     Usage (iDeviceChemistry)
    0xB1, 0x23,       //     Feature
    // CapacityMode (feature)
    0x85, 0x16,       //     Report ID (22)
    0x09, 0x2C,       //     Usage (CapacityMode)
    0xB1, 0x23,       //     Feature
    // FullChargeCapacity (feature)
    0x85, 0x0E,       //     Report ID (14)
    0x09, 0x67,       //     Usage (FullChargeCapacity)
    0x75, 0x10,       //     Report Size (16)
    0x95, 0x01,       //     Report Count (1)
    0x67, 0x01, 0x10, 0x10, 0x00, // Unit (AmpSec)
    0x55, 0x00,       //     Unit Exponent (0)
    0xB1, 0x83,       //     Feature (Const, Var, Abs, Volatile, Bitfield)
    // DesignCapacity (feature)
    0x85, 0x0F,       //     Report ID (15)
    0x09, 0x83,       //     Usage (DesignCapacity)
    0xB1, 0x83,       //     Feature
    // RemainingCapacity (input + feature)
    0x85, 0x0C,       //     Report ID (12)
    0x09, 0x66,       //     Usage (RemainingCapacity)
    0x81, 0xA3,       //     Input (Const, Var, Abs, Volatile, Bitfield)
    0x09, 0x66,       //     Usage (RemainingCapacity)
    0xB1, 0xA3,       //     Feature
    // RemainingCapacityLimit (feature)
    0x85, 0x10,       //     Report ID (16)
    0x09, 0x29,       //     Usage (RemainingCapacityLimit)
    0xB1, 0xA2,       //     Feature (Data, Var, Abs, Volatile, Bitfield)
    // WarningCapacityLimit (feature)
    0x85, 0x11,       //     Report ID (17)
    0x09, 0x8C,       //     Usage (WarningCapacityLimit)
    0xB1, 0xA2,       //     Feature
    // ManufacturerDate (feature)
    0x85, 0x09,       //     Report ID (9)
    0x09, 0x85,       //     Usage (ManufacturerDate)
    0x75, 0x10,       //     Report Size (16)
    0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65534)
    0xB1, 0xA3,       //     Feature
    // RunTimeToEmpty (input + feature)
    0x85, 0x0D,       //     Report ID (13)
    0x09, 0x68,       //     Usage (RunTimeToEmpty)
    0x81, 0xA3,       //     Input
    0x09, 0x68,       //     Usage (RunTimeToEmpty)
    0xB1, 0xA3,       //     Feature
    // CycleCount (input + feature)
    0x85, 0x14,       //     Report ID (20)
    0x09, 0x6B,       //     Usage (CycleCount)
    0x75, 0x10,       //     Report Size (16)
    0x15, 0x00,       //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65534)
    0x81, 0x22,       //     Input (Data, Var, Abs)
    0x09, 0x6B,       //     Usage (CycleCount)
    0xB1, 0xA2,       //     Feature
    0x05, 0x84,       //     Usage Page (Power Device)
    // Temperature (input + feature)
    0x85, 0x0A,       //     Report ID (10)
    0x09, 0x36,       //     Usage (Temperature)
    0x75, 0x10,       //     Report Size (16)
    0x67, 0x01, 0x00, 0x01, 0x00, // Unit (Kelvin)
    0x55, 0x00,       //     Unit Exponent (0)
    0x81, 0xA3,       //     Input
    0x09, 0x36,       //     Usage (Temperature)
    0xB1, 0xA3,       //     Feature
    // Voltage (input + feature)
    0x85, 0x0B,       //     Report ID (11)
    0x09, 0x30,       //     Usage (Voltage)
    0x15, 0x00,       //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65535)
    0x67, 0x21, 0xD1, 0xF0, 0x00, // Unit (Centivolts)
    0x55, 0x05,       //     Unit Exponent (5)
    0x81, 0xA3,       //     Input
    0x09, 0x30,       //     Usage (Voltage)
    0xB1, 0xA3,       //     Feature
    // PresentStatus bitfield (input + feature)
    0x09, 0x02,       //     Usage (PresentStatus)
    0xA1, 0x02,       //     Collection (Logical)
    0x85, 0x07,       //       Report ID (7)
    0x05, 0x85,       //       Usage Page (Battery System)
    0x09, 0x44,       //       Usage (Charging)
    0x75, 0x01,       //       Report Size (1)
    0x15, 0x00,       //       Logical Minimum (0)
    0x25, 0x01,       //       Logical Maximum (1)
    0x81, 0xA3,       //       Input
    0x09, 0x44,       //       Usage (Charging)
    0xB1, 0xA3,       //       Feature
    0x09, 0x45,       //       Usage (Discharging)
    0x81, 0xA3,       //       Input
    0x09, 0x45,       //       Usage (Discharging)
    0xB1, 0xA3,       //       Feature
    0x09, 0xD0,       //       Usage (ACPresent)
    0x81, 0xA3,       //       Input
    0x09, 0xD0,       //       Usage (ACPresent)
    0xB1, 0xA3,       //       Feature
    0x05, 0x84,       //       Usage Page (Power Device)
    0x09, 0x69,       //       Usage (ShutdownImminent)
    0x81, 0xA3,       //       Input
    0x09, 0x69,       //       Usage (ShutdownImminent)
    0xB1, 0xA3,       //       Feature
    0x95, 0x04,       //       Report Count (4) - padding bits to byte-align
    0x81, 0x01,       //       Input (Const, Array)
    0xB1, 0x01,       //       Feature (Const, Array)
    0xC0,             //     End Collection (PresentStatus)
    0xC0,             //   End Collection (PowerSummary)
    0xC0              // End Collection (Application)
};
static_assert(sizeof(ups_report_descriptor_bytes) == UPS_REPORT_DESC_LEN,
              "UPS HID report descriptor length must match UPS_REPORT_DESC_LEN");

namespace {

uint8_t  g_status   = UPS_ST_DISCHARGING;
uint16_t g_remaining = UPS_FULL_CAPACITY;
uint16_t g_runtime   = 0;
uint32_t g_last_tick_ms = 0;
bool     g_real_seen   = false;

uint16_t ups_copy8(uint8_t *buffer, uint16_t reqlen, uint8_t value) {
    if (reqlen < 1) return 0;
    buffer[0] = value;
    return 1;
}

uint16_t ups_copy16(uint8_t *buffer, uint16_t reqlen, uint16_t value) {
    if (reqlen < 2) return 0;
    buffer[0] = (uint8_t) (value & 0xFF);
    buffer[1] = (uint8_t) (value >> 8);
    return 2;
}

} // namespace

uint8_t ups_hid_instance() {
    // HID instances follow interface order: gamepad = 0, wake keyboard (when
    // present) = 1, UPS = 1 or 2. The kbd interface is present exactly when
    // usb_kbd_iface_needed() says so (see usb_descriptors.cpp).
    return usb_kbd_iface_needed(get_config()) ? 2u : 1u;
}

uint8_t const *ups_report_descriptor() {
    return ups_report_descriptor_bytes;
}

uint16_t ups_get_report(uint8_t report_id, uint8_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void) report_type; // feature and input reports carry the same values
    switch (report_id) {
        case UPS_RID_IPRODUCT:          return ups_copy8(buffer, reqlen, 2);    // STRID_PRODUCT
        case UPS_RID_ISERIAL:           return ups_copy8(buffer, reqlen, 3);    // STRID_SERIAL
        case UPS_RID_IMANUFACTURER:     return ups_copy8(buffer, reqlen, 1);    // STRID_MANUFACTURER
        case UPS_RID_IDEVICECHEMISTRY:  return ups_copy8(buffer, reqlen, 5);    // "LiP"
        case UPS_RID_CAPACITYMODE:      return ups_copy8(buffer, reqlen, 0);    // 0 = mAh
        case UPS_RID_PRESENTSTATUS:     return ups_copy8(buffer, reqlen, g_status);
        case UPS_RID_FULLCHARGE:        return ups_copy16(buffer, reqlen, UPS_FULL_CAPACITY);
        case UPS_RID_DESIGNCAPACITY:    return ups_copy16(buffer, reqlen, UPS_DESIGN_CAPACITY);
        case UPS_RID_REMAINING:         return ups_copy16(buffer, reqlen, g_remaining);
        case UPS_RID_REMNCAPLIMIT:      return ups_copy16(buffer, reqlen, UPS_CRITICAL_LIMIT);
        case UPS_RID_WARNCAPLIMIT:      return ups_copy16(buffer, reqlen, UPS_WARN_LIMIT);
        case UPS_RID_MANUFACTUREDATE:   return ups_copy16(buffer, reqlen, UPS_MANUFACTURE_DATE);
        case UPS_RID_RUNTIMETOEMPTY:    return ups_copy16(buffer, reqlen, g_runtime);
        case UPS_RID_CYCLECOUNT:        return ups_copy16(buffer, reqlen, 0);
        case UPS_RID_TEMPERATURE:       return ups_copy16(buffer, reqlen, 300);  // Kelvin
        case UPS_RID_VOLTAGE:           return ups_copy16(buffer, reqlen, 3600); // centivolts
        default:
            return 0;
    }
}

void ups_battery_tick() {
    const auto &cfg = get_config();
    if (cfg.battery_mode == 0) return;

    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - g_last_tick_ms < 1000) return;
    g_last_tick_ms = now;

    uint8_t level  = UPS_SAFETY_FLOOR;
    uint8_t status = 0;

    // The controller's REAL charge state (payload [52]: high nibble 1=charging,
    // 2=full, else on battery). Both modes sync the Windows charging state to
    // it, so the tray icon shows "charging" while the DualSense is plugged in.
    const uint8_t b    = interrupt_in_data[52];
    const uint8_t st   = (b >> 4) & 0x0F;
    const bool realChg = (st == 0x1) || (st == 0x2); // charging or full
    const bool realData = b != 0;

    if (cfg.battery_mode == 2) {
        // Simulated: fixed level from the portal. Clamped to the safety floor
        // so Windows never sees a critical system battery and starts
        // hibernate/shutdown actions on the emulated UPS.
        level = cfg.battery_fake;
        if (level > 100) level = 100;
        if (level < UPS_SAFETY_FLOOR) level = UPS_SAFETY_FLOOR;
        // Charging state SYNCs with the real controller: if the DualSense is
        // charging or full, Windows shows the simulated battery as charging on
        // AC; otherwise it shows a battery running on battery power.
        status = (realData && realChg) ? (UPS_ST_CHARGING | UPS_ST_AC_PRESENT)
                                       : UPS_ST_DISCHARGING;
        g_real_seen = false;
    } else {
        // Real: mirror the controller's Bluetooth battery byte (payload [52]:
        // low nibble = level 0-10 (x10 = %), high nibble = 1 charging, 2 full).
        if (b == 0) return; // no report yet / unknown - keep the last values
        g_real_seen = true;
        uint8_t lvl10 = b & 0x0F;
        if (lvl10 > 10) lvl10 = 10;
        level = (uint8_t) (lvl10 * 10u);
        if (level > 100) level = 100;
        if (level < UPS_SAFETY_FLOOR) level = UPS_SAFETY_FLOOR;
        if (realChg) {
            status = UPS_ST_CHARGING | UPS_ST_AC_PRESENT;
        } else {
            status = UPS_ST_DISCHARGING;
        }
    }

    g_remaining = level;
    g_status    = status;
    g_runtime   = (uint16_t) (level * 60u); // cosmetic: 1% ~ 1 minute

    const uint8_t inst = ups_hid_instance();
    if (!tud_hid_n_ready(inst)) return;

    uint8_t payload[2];
    payload[0] = (uint8_t) (g_remaining & 0xFF);
    payload[1] = (uint8_t) (g_remaining >> 8);
    tud_hid_n_report(inst, UPS_RID_REMAINING, payload, 2);
    payload[0] = (uint8_t) (g_runtime & 0xFF);
    payload[1] = (uint8_t) (g_runtime >> 8);
    tud_hid_n_report(inst, UPS_RID_RUNTIMETOEMPTY, payload, 2);
    payload[0] = g_status;
    tud_hid_n_report(inst, UPS_RID_PRESENTSTATUS, payload, 1);
    // Temperature / Voltage / CycleCount (like HidBattery) - Windows 11 build
    // 29550+ parses these; harmless on older builds.
    payload[0] = 300 & 0xFF;      // Kelvin
    payload[1] = 300 >> 8;
    tud_hid_n_report(inst, UPS_RID_TEMPERATURE, payload, 2);
    payload[0] = 3600 & 0xFF;     // centivolts
    payload[1] = 3600 >> 8;
    tud_hid_n_report(inst, UPS_RID_VOLTAGE, payload, 2);
    payload[0] = 0;               // cycle count
    payload[1] = 0;
    tud_hid_n_report(inst, UPS_RID_CYCLECOUNT, payload, 2);
}
