//
// macro.h - controller chord / touchpad gesture -> keyboard combo macros.
//
// Generalises ps_shortcut.cpp, which was a single hardcoded macro (PS short ->
// Win+G, PS long -> Win+Tab) and already proved the whole shape: edge detection
// on the BT input path, short-vs-long discrimination, and a deferred key
// release on HID instance 1.
//
// STORAGE SPLIT, and it is the important design decision:
//   - The macro TABLE is DEVICE-GLOBAL, in its own flash sector. Definitions
//     are not duplicated per slot and survive a browser reset or a different PC.
//   - Config_body carries only macro_enable, a 32-bit mask. Slots therefore
//     select which subset of the shared table is live, which is what makes
//     per-game macro sets work through the existing Playnite slot automation.
// The definitions CANNOT live only in the portal: the dongle runs standalone
// with nothing attached, so it must hold what each enabled bit means.
//

#ifndef DS5_BRIDGE_MACRO_H
#define DS5_BRIDGE_MACRO_H

#include <cstdint>

#include "config.h"

constexpr uint8_t  MACRO_COUNT     = 32; // matches the 32 bits of macro_disable

// All-disabled sentinel for Config_body::macro_disable. The mask is stored
// INVERTED so an old slot's 0xFF fill defaults to "no macros"; see config.h for
// why a range clamp cannot do that job for a bitmap.
constexpr uint32_t MACRO_NONE_ENABLED = 0xFFFFFFFFu;
static inline bool macro_is_enabled(uint32_t disable_mask, uint8_t idx) {
    return idx < MACRO_COUNT && (disable_mask & (1u << idx)) == 0u;
}
static inline bool macro_any_enabled(uint32_t disable_mask) {
    return disable_mask != MACRO_NONE_ENABLED;
}

// Is the wake keyboard INTERFACE present in the configuration descriptor?
//
// SINGLE SOURCE OF TRUTH, and it has to be: usb_descriptors.cpp decides the
// interface count from this, and slot_activate/ENUM_FIELDS decide whether a
// change needs a USB re-enumeration. Testing enable_wake, ps_shortcut_enabled
// and the macro mask INDEPENDENTLY - which is what those sites used to do - is
// wrong in both directions. With wake already on, enabling a macro changes
// nothing about the descriptor, yet it forced a reconnect; and that fires on
// every Playnite slot switch between two wake-on profiles with different macro
// sets.
//
// NOTE enable_wake still needs its own re-enumeration test elsewhere: beyond
// this interface it also sets bcdUSB 2.1, the BOS descriptor and the
// REMOTE_WAKEUP attribute bit.
static inline bool usb_kbd_iface_needed(const Config_body &c) {
    return c.enable_wake || c.ps_shortcut_enabled || macro_any_enabled(c.macro_disable);
}
constexpr uint8_t  MACRO_KEYS      = 4;  // keys per combo, excluding nothing - modifiers count
constexpr uint8_t  MACRO_LABEL_LEN = 16; // portal display name, stored on device

// --- Output encoding ---------------------------------------------------------
// Keys are HID USAGE CODES throughout, including modifiers (0xE0 LeftCtrl ...
// 0xE7 RightGUI). One uniform namespace means no separate modifier field and no
// ambiguity about ordering between a modifier and a key: macro_play() folds any
// 0xE0-0xE7 usage into the boot keyboard's modifier byte as it walks the list.
//
// keys[] is in PRESS order. rel_order is the RELEASE order as a permutation,
// 2 bits per slot (slot i released at position ((rel_order >> (2*i)) & 3)).
// That is what distinguishes Alt+Tab (release Tab, then Alt) from a combo where
// the modifier lifts first.
//
// Reverse-press order is the overwhelmingly common case; the recorder should
// fall back to it when a capture produces an ambiguous or implausible ordering.
constexpr uint8_t HID_USAGE_MOD_FIRST = 0xE0;
constexpr uint8_t HID_USAGE_MOD_LAST  = 0xE7;

// Milliseconds between successive keyboard reports during playback. Long enough
// that a host processes each transition, short enough to feel instant.
constexpr uint8_t MACRO_STEP_MS = 15;

// --- Trigger encoding --------------------------------------------------------
// gesture == GESTURE_NONE means this is a chord macro and `chord` is a logical
// button mask from input_buttons.h. Otherwise `chord` is unused and the entry
// fires on a touchpad swipe.
constexpr uint8_t GESTURE_NONE = 0;
// Packed: bits 0-1 direction, bit 2 start zone, bit 3 finger count.
enum : uint8_t {
    GEST_DIR_UP     = 0u << 0,
    GEST_DIR_DOWN   = 1u << 0,
    GEST_DIR_LEFT   = 2u << 0,
    GEST_DIR_RIGHT  = 3u << 0,
    GEST_DIR_MASK   = 3u << 0,
    GEST_ZONE_RIGHT = 1u << 2, // swipe STARTED on the right half of the pad
    GEST_TWO_FINGER = 1u << 3,
    GEST_MOTION     = 1u << 4, // MOTION gesture: motion[] holds the template
    GEST_VALID      = 1u << 7, // set on every real gesture so the byte is non-zero
};

// --- Motion gestures ---------------------------------------------------------
// A THIRD trigger kind: hold a gate button, move the controller, release. The
// gate is what removes start/end detection - without it an always-on recogniser
// competes with gyro aiming and fires during normal play.
//
// `chord` carries the GATE mask for a motion entry. It is otherwise unused on a
// non-chord entry, and reusing it means find_entry()/best_chord() already skip
// these records (they skip anything with gesture != GESTURE_NONE), so the gate
// button cannot be stolen by chord matching.
//
// The template is a sequence of STROKE directions, 4-way, 2 bits each. Eight
// directions were tried first and failed on hardware: at 22.5 degrees per
// sector a hand cannot hold an axis, and one up-flick quantised as
// up/up-left/up/up-left to the length ceiling.
constexpr uint8_t MACRO_MOTION_MAX   = 8;  // codes; a real gesture is 1-4
constexpr uint8_t MACRO_MOTION_BYTES = 2;  // 8 codes x 2 bits
enum : uint8_t {
    MOTION_RIGHT = 0,
    MOTION_UP    = 1,
    MOTION_LEFT  = 2,
    MOTION_DOWN  = 3,
};
// Default step threshold in raw gyro counts. Per-entry because it is CALIBRATED
// from the user's own motion - a constant chosen without hardware produced a
// code storm, which is what sent the portal prototype back twice.
constexpr uint16_t MACRO_MOTION_STEP_DEFAULT = 1800;

enum : uint8_t {
    MACRO_FLAG_LONG_PRESS = 1u << 0, // fire at hold_cs, not on release
};

// Default long-press threshold, centiseconds. 750 ms, matching ps_shortcut.
constexpr uint8_t MACRO_HOLD_CS_DEFAULT = 75;

struct __attribute__((packed)) MacroEntry {
    uint32_t chord;       // logical button mask (input_buttons.h); 0 if gesture
    uint8_t  gesture;     // GESTURE_NONE or GEST_* bits
    uint8_t  flags;       // MACRO_FLAG_*
    uint8_t  hold_cs;     // long-press threshold, centiseconds (0 -> default)
    uint8_t  keys[MACRO_KEYS]; // HID usages in PRESS order, 0 = unused
    uint8_t  rel_order;   // release permutation, 2 bits per slot
    // --- appended for motion gestures; absent from rec_len 28 tables ---
    uint8_t  motion[MACRO_MOTION_BYTES]; // 2 bits per stroke, index 0 first
    uint8_t  motion_len;  // strokes used, 0 on every non-motion entry
    uint16_t motion_step; // raw gyro counts per stroke; 0 -> default
    // No reserved padding: MacroTable.rec_len makes the record self-describing,
    // so a later firmware with a LARGER record still reads today's tables (the
    // same mechanism SlotRecordV2.body_len uses to survive Config_body growth).
};
static_assert(sizeof(MacroEntry) == 17, "MacroEntry size is part of the flash format");

static inline uint8_t macro_motion_code(const MacroEntry &e, uint8_t i) {
    if (i >= MACRO_MOTION_MAX) return 0;
    return (uint8_t) ((e.motion[i >> 2] >> ((i & 3u) * 2u)) & 3u);
}
static inline void macro_motion_set_code(MacroEntry &e, uint8_t i, uint8_t code) {
    if (i >= MACRO_MOTION_MAX) return;
    const uint8_t sh = (uint8_t) ((i & 3u) * 2u);
    e.motion[i >> 2] = (uint8_t) ((e.motion[i >> 2] & ~(3u << sh)) | ((code & 3u) << sh));
}
static inline bool macro_is_motion(const MacroEntry &e) {
    return (e.gesture & GEST_MOTION) != 0 && e.motion_len > 0;
}

struct __attribute__((packed)) MacroRecord {
    MacroEntry entry;
    uint8_t    label[MACRO_LABEL_LEN]; // NUL-padded, portal display only
};
static_assert(sizeof(MacroRecord) == 33);

// Whole table + header, rewritten as one sector image like the slot sectors.
constexpr uint32_t MACRO_MAGIC   = 0x4D355344; // "DS5M"
constexpr uint8_t  MACRO_FORMAT  = 1;

struct __attribute__((packed)) MacroTable {
    uint32_t    magic;
    uint8_t     format;    // MACRO_FORMAT
    uint8_t     count;     // MACRO_COUNT at write time; extra entries default
    uint16_t    rec_len;   // sizeof(MacroRecord) at write time
    MacroRecord rec[MACRO_COUNT];
    uint32_t    crc32;     // over rec[0..count) using rec_len bytes each
};
static_assert(sizeof(MacroTable) <= 4096, "macro table must fit one flash sector");

// --- API ---------------------------------------------------------------------

// Load the table from flash into the RAM working image. Missing/invalid sector
// (virgin flash on every existing device) yields an all-empty table - no
// migration and no flash_nuke on upgrade.
void macro_load();

// Edit the RAM image, then commit once. Per-entry writes with a single explicit
// commit avoid erasing the sector 32 times while the portal saves a list.
bool macro_get(uint8_t idx, MacroRecord &out);
bool macro_set_entry(uint8_t idx, const MacroRecord &rec);
bool macro_commit();

// Called from main.cpp's on_bt_data for every input report, with the same
// `data + 3` pointer ps_shortcut_tick already receives.
void macro_on_input(const uint8_t *report, uint16_t len);

// Service deferred work: long-press thresholds that expire with no new report,
// and the multi-step playback walk. Call from the main loop.
void macro_task();

// Drop all pending state and release any keys still down. Called on controller
// disconnect (beside ps_shortcut_reset) and on the host-suspend edge, so a
// combo caught mid-playback cannot leave a modifier latched at the host across
// sleep - the keyboard-side analogue of state_release_for_suspend().
void macro_reset();

// Portal record mode. While suspended, chords are still decoded and reported
// but never fire, so capturing a chord that matches an already-enabled macro
// cannot type into the portal while the user is recording it.
void macro_suspend(bool on);

// True while a motion gate is held and strokes are being accumulated. main.cpp
// suppresses gyro-to-stick aiming while this is set, so performing a gesture
// does not also swing the aim - the same idea as gyro_mode 4 pausing on a
// touchpad touch.
bool macro_motion_capturing();

// True while the engine holds keys down or has queued playback steps. wake.cpp
// uses it to avoid interleaving its F15 keystroke with a macro on the shared
// keyboard instance.
bool macro_busy();

#endif // DS5_BRIDGE_MACRO_H
