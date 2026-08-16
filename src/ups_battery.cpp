//
// ups_battery.cpp - HID UPS Battery interface implementation.
//

#include "ups_battery.h"

#include <cstdint>

#include "tusb.h"
#include "config.h"
#include "macro.h"
#include "bt.h"
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

// Smoothing state: the percentage actually reported to Windows eases toward
// the target rather than stepping, when battery_smooth is enabled.
uint8_t  g_smooth_current = UPS_FULL_CAPACITY;
bool     g_smooth_seeded  = false;   // first real report seeds the smooth value

// Voltage-blend state. Polling is rate-limited and cooperative: we record the
// response generation, issue a factory-test voltage read, and consume the
// shadowed result (never the shared feature_data map) once it advances.
uint16_t g_volt_cache_mv  = 0;
uint32_t g_volt_next_poll_ms = 0;   // throttle to ~30 s
bool     g_volt_pending   = false;  // a request is in flight
uint32_t g_volt_since_gen = 0;      // generation before the request was issued

// Auto-calibration persistence throttle (RAM anchors -> flash). config_save()
// erases+programs the config sector, which must not happen on every voltage
// poll (up to 10/s at a 100 ms interval).
bool     g_calib_dirty        = false;
uint32_t g_calib_last_save_ms = 0;

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

// Voltage -> percentage. When auto-calibration (battery_calib_enable) has
// sampled at least two BT levels, the mapping is a piecewise-linear fit through
// the observed (voltage, level*10) anchors, sorted by voltage - so it tracks
// THIS battery's real discharge curve (age, chemistry, habits) instead of the
// generic one. Otherwise (or when calibration is off) it falls back to the
// built-in Li-ion table below. Values outside the used table clamp to 0/100.
static uint8_t ups_volt_to_pct(uint16_t volt_mv) {
    // {voltage_mV, percent} anchor points on a typical Li-ion curve.
    static const struct { uint16_t mv; uint8_t pct; } table[] = {
        {3000, 0}, {3300, 3}, {3400, 6}, {3500, 10}, {3600, 18},
        {3700, 32}, {3800, 48}, {3900, 65}, {4000, 82}, {4100, 93}, {4200, 100},
    };

    const auto &cfg = get_config();
    if (cfg.battery_calib_enable) {
        // Collect the sampled anchors: {observed mV, level*10 %}. n <= 11.
        struct Pt { uint16_t mv; uint8_t pct; } pts[11];
        uint8_t n = 0;
        for (uint8_t lvl = 0; lvl < 11; lvl++) {
            const uint16_t mv = cfg.battery_calib_volt[lvl];
            if (mv >= 2500 && mv <= 4500) { pts[n].mv = mv; pts[n].pct = (uint8_t)(lvl * 10u); n++; }
        }
        if (n >= 2) {
            // Sort by voltage (insertion sort; n <= 11, and noisy observations
            // can make levels non-monotonic).
            for (uint8_t i = 1; i < n; i++) {
                const Pt key = pts[i];
                int8_t j = (int8_t) i - 1;
                while (j >= 0 && pts[j].mv > key.mv) { pts[j + 1] = pts[j]; j--; }
                pts[j + 1] = key;
            }
            if (volt_mv <= pts[0].mv) return pts[0].pct;
            if (volt_mv >= pts[n - 1].mv) return pts[n - 1].pct;
            for (uint8_t i = 1; i < n; i++) {
                if (volt_mv <= pts[i].mv) {
                    const int32_t span = (int32_t) pts[i].mv - (int32_t) pts[i - 1].mv;
                    int32_t pct = pts[i - 1].pct;
                    if (span > 0) {
                        const int32_t frac = ((int32_t) volt_mv - (int32_t) pts[i - 1].mv) *
                                             ((int32_t) pts[i].pct - (int32_t) pts[i - 1].pct);
                        pct += frac / span;
                    }
                    return (uint8_t) pct;
                }
            }
            return pts[n - 1].pct;
        }
    }

    if (volt_mv <= table[0].mv) return table[0].pct;
    if (volt_mv >= table[10].mv) return table[10].pct;
    for (int i = 1; i < 11; i++) {
        if (volt_mv <= table[i].mv) {
            const uint32_t span = table[i].mv - table[i-1].mv;
            const int32_t frac = ((int32_t)(volt_mv - table[i-1].mv) * (table[i].pct - table[i-1].pct));
            uint8_t pct = table[i-1].pct;
            if (span > 0) pct += (uint8_t) (frac / (int32_t) span);
            return pct;
        }
    }
    return 100;
}

// Blend the controller's coarse BT level (0-10) with the factory-test voltage.
// weight (0-100) selects how much the VOLTAGE drives the result: 100 = voltage
// only, 0 = level only. The voltage mapping is a real discharge curve, so an
// aged battery whose voltage reads low is reflected in a lower blended %.
// Falls back to level*10 when no voltage is available.
uint8_t ups_blend_level(uint8_t level10, uint16_t volt_mv, uint8_t weight) {
    const uint8_t lvl_pct = (uint8_t) (level10 * 10u);
    if (volt_mv == 0) {
        return lvl_pct;
    }
    const uint8_t volt_pct = ups_volt_to_pct(volt_mv);
    if (weight >= 100) return volt_pct;
    if (weight == 0) return lvl_pct;
    // weight% voltage + (100-weight)% level, rounded.
    uint32_t pct = ((uint32_t) volt_pct * weight + (uint32_t) lvl_pct * (100u - weight) + 50u) / 100u;
    if (pct < 5) pct = 5;
    if (pct > 100) pct = 100;
    return (uint8_t) pct;
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
        //
        // DEMO DISCHARGE: when battery_fake is exactly 100 the firmware ramps
        // the reported level down from 100% at 30% per minute (reaching 70%
        // after exactly one minute) and holds there. This lets a user verify
        // that Windows' HID UPS battery display tracks live input reports
        // (tray percentage + power settings) without touching the portal.
        // Any other battery_fake value behaves as a fixed level.
        level = cfg.battery_fake;
        if (level > 100) level = 100;
        if (level < UPS_SAFETY_FLOOR) level = UPS_SAFETY_FLOOR;
        if (cfg.battery_fake == 100) {
            // 30% per minute = 0.5% per 1 Hz tick. Track in tenths to avoid
            // float: start 1000, step -5, floor 700 (70.0%).
            static uint16_t demo_tenths = 1000;
            if (demo_tenths > 700) {
                if (demo_tenths >= 5) demo_tenths -= 5;
                level = (uint8_t) (demo_tenths / 10);
            } else {
                level = 70;
            }
        }
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

        // Optional refinement 1: blend the factory-test battery voltage (mV)
        // into the coarse 10% level. Cooperative polling: record the response
        // generation, issue the factory test command, and on later ticks accept
        // the shadowed cache only once the generation advanced. Retail
        // controllers may reject the command; the cache then stays stale/0 and
        // we keep the last good value (or fall back to level*10). Polling is
        // throttled to 30 s so the test command cannot disturb the link.
        if (cfg.battery_volt_blend) {
            const uint32_t now = to_ms_since_boot(get_absolute_time());
            if (g_volt_pending) {
                uint16_t mv = 0;
                if (bt_battery_volt_take(g_volt_since_gen, &mv)) {
                    g_volt_cache_mv = mv;
                    g_volt_pending  = false;
                    // Auto-calibration: pair the fresh factory-test voltage
                    // with the BT level reported right now. The level the
                    // controller reports at that voltage is ground truth for
                    // THAT battery; the EMA anchor for that level creeps
                    // toward the observed voltage, so the mapping learns the
                    // real discharge curve instead of a generic Li-ion one.
                    // Only sample plausible voltages (3000-4500 mV) - a bogus
                    // 0xFFFF from a rejected command must never corrupt an
                    // anchor. Config is saved to flash throttled (see below),
                    // so anchors survive reboot without wearing flash.
                    if (cfg.battery_calib_enable && mv >= 3000 && mv <= 4500) {
                        auto &body = get_config();
                        uint16_t anchor = body.battery_calib_volt[lvl10];
                        if (anchor == 0) {
                            anchor = mv;                       // first sample: seed
                        } else {
                            // EMA with alpha 1/8, integer-rounded, no drift:
                            // anchor += (mv - anchor + 4) / 8 for both signs.
                            const int32_t diff = (int32_t) mv - (int32_t) anchor;
                            anchor = (uint16_t) ((int32_t) anchor + ((diff >= 0) ? (diff + 4) / 8 : (diff - 4) / 8));
                        }
                        body.battery_calib_volt[lvl10] = anchor;
                        g_calib_dirty = true;
                    }
                } else if (now - g_volt_next_poll_ms > 3000) {
                    g_volt_pending = false; // timeout: command unanswered
                }
            }
            // Throttled persistence: anchor changes are RAM-only while they
            // accumulate; write to flash at most every 10 s while dirty, so a
            // single session of use doesn't wear the flash sector with a write
            // per voltage poll (which can be 100 ms apart).
            if (g_calib_dirty && now - g_calib_last_save_ms >= 10000) {
                g_calib_dirty = false;
                g_calib_last_save_ms = now;
                config_save();
            }
            if (!g_volt_pending && now >= g_volt_next_poll_ms) {
                // Interval from config, stored in deciseconds (0.1 s):
                // 1 = 100 ms .. 3000 = 300 s, default 300 (30 s). Fresh-flash
                // value lands on the default via config_valid().
                uint32_t ds = cfg.battery_volt_poll_s;
                if (ds < 1) ds = 1;
                uint32_t interval_ms = ds * 100u;
                g_volt_next_poll_ms = now + interval_ms;
                g_volt_pending = true;
                g_volt_since_gen = bt_battery_volt_gen();
                // Factory test command ANALOG_DATA(4)/BATTERY(3), mirroring
                // dualsense-tester. set_feature_data() appends a CRC32 trailer
                // and REQUIRES a >= 5-byte payload (len-4 must be >= 1), so pad
                // to the same length DSE's unlock command uses (59 bytes).
                uint8_t cmd[59]{};
                cmd[0] = 0x04; // DualSenseTestDeviceId.ANALOG_DATA
                cmd[1] = 0x03; // DualSenseTestActionId.BATTERY
                set_feature_data(0x80, cmd, sizeof(cmd));
            }
            level = ups_blend_level(lvl10, g_volt_cache_mv, cfg.battery_volt_weight);
        } else {
            g_volt_cache_mv = 0;
            g_volt_pending  = false;
            level = (uint8_t) (lvl10 * 10u);
        }
        if (level > 100) level = 100;
        if (level < UPS_SAFETY_FLOOR) level = UPS_SAFETY_FLOOR;

        // Optional refinement 2: smooth the reported percentage toward the
        // target so the tray number glides instead of snapping. Step is a
        // fraction of the REMAINING gap (ease-out), so a big jump (e.g. a
        // battery level change) moves quickly at first and settles gently,
        // and a small drift (voltage blend) crawls without visible jumps.
        // At 1 Hz ticks, 25% of the gap per tick reaches the target in ~4-5
        // ticks while staying smooth. First real report seeds to the target.
        if (cfg.battery_smooth) {
            const uint8_t target = level;
            if (g_smooth_seeded == false) {
                g_smooth_seeded = true;
                g_smooth_current = target;
            } else if (g_smooth_current != target) {
                const int diff = (int) target - (int) g_smooth_current;
                int step = (diff >= 0) ? ((diff + 3) / 4) : ((diff - 3) / 4);
                if (step == 0) step = (diff >= 0) ? 1 : -1;
                const int next = (int) g_smooth_current + step;
                g_smooth_current = (uint8_t) ((next < target) == (diff >= 0) ? next : target);
            }
            level = g_smooth_current;
        } else {
            g_smooth_current = level;
            g_smooth_seeded  = false;
        }

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

// Exposed for the portal (read-only diagnostic 0x92): last factory-test
// battery voltage in mV. Defined outside the anonymous namespace but the
// variable is file-static; read it through this accessor.
uint16_t ups_last_battery_voltage_mv() {
    return g_volt_cache_mv;
}

uint8_t ups_calib_sample_count() {
    const auto &body = get_config();
    uint8_t n = 0;
    for (uint8_t i = 0; i < 11; i++) {
        const uint16_t mv = body.battery_calib_volt[i];
        if (mv >= 2500 && mv <= 4500) n++;
    }
    return n;
}

void ups_calib_save_now() {
    g_calib_dirty = false;
    config_save();
}
