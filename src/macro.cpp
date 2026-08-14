//
// macro.cpp - chord / gesture -> keyboard combo engine.
//
// Generalises ps_shortcut.cpp. See docs/MACRO-ARCHITECTURE.md for the storage
// split and the reasoning behind the inverted enable mask.
//

#include "macro.h"

#include <cstdio>
#include <cstring>

#include "config.h"
#include "flash_map.h"
#include "input_buttons.h"
#include "utils.h"
#include "wake.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/time.h"
#include "tusb.h"

// Instance 1 is the wake keyboard. It is a CONSTANT in both identities - the
// interface layout is byte-identical whether or not the idle USB identity is
// active, which is the invariant that makes the v1.18.9 report-storm
// arithmetically impossible. Never derive this from a runtime flag.
#define MACRO_KBD_INSTANCE 1

// A bit that drops out of one BT report is not a release. ps_shortcut.cpp uses
// the same 50 ms hold for exactly this reason.
constexpr uint32_t MACRO_DEBOUNCE_MS = 50;

// Swipe classification thresholds. Pad is 1920 x 1080.
constexpr uint32_t GEST_MIN_MS = 40;
constexpr uint32_t GEST_MAX_MS = 600;
constexpr int32_t  GEST_MIN_DX = 300;
constexpr int32_t  GEST_MIN_DY = 200;

// Flash image is page-granular; sizeof(MacroTable) is 908.
constexpr uint32_t MACRO_IMAGE_BYTES =
    ((sizeof(MacroTable) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
static_assert(MACRO_IMAGE_BYTES <= FLASH_SECTOR_SIZE);

// ============================== state ======================================

static MacroTable g_table;          // RAM working image
static bool       g_loaded    = false;
static bool       g_suspended = false;

// Chord tracking
static uint32_t g_stable       = 0;      // debounced held mask
static uint32_t g_off_since[32] = {0};   // per-bit fall time, 0 = not falling
static uint32_t g_chord        = 0;      // armed chord value, 0 = none
static uint32_t g_chord_start  = 0;
static bool     g_chord_fired  = false;  // long fired, or short fired on press

// Touch tracking
static bool     g_touch_down = false;
static bool     g_touch_two  = false;
static uint16_t g_touch_x0   = 0;
static uint16_t g_touch_y0   = 0;
static uint16_t g_touch_lx   = 0;   // last sample while STILL DOWN
static uint16_t g_touch_ly   = 0;
static uint32_t g_touch_t0   = 0;

// Playback walk
static uint8_t  g_pb_keys[MACRO_KEYS];
static uint8_t  g_pb_rel  = 0;   // release permutation
static uint8_t  g_pb_n    = 0;   // key count
static uint8_t  g_pb_step = 0;   // 0 .. 2n, 2n means "done after this"
static bool     g_pb_busy = false;
static uint32_t g_pb_next = 0;

static inline uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

// ============================== storage ====================================

static uint32_t table_crc(const MacroTable &t) {
    return crc32(reinterpret_cast<const uint8_t *>(t.rec), sizeof(t.rec));
}

static void table_defaults(MacroTable &t) {
    memset(&t, 0, sizeof(t));
    t.magic   = MACRO_MAGIC;
    t.format  = MACRO_FORMAT;
    t.count   = MACRO_COUNT;
    t.rec_len = sizeof(MacroRecord);
    t.crc32   = table_crc(t);
}

void macro_load() {
    const auto *flashed =
        reinterpret_cast<const MacroTable *>(XIP_BASE + MACRO_FLASH_OFFSET);

    // Virgin flash on every existing device: no magic, so fall to an empty
    // table. No migration and no flash_nuke on upgrade.
    if (flashed->magic != MACRO_MAGIC || flashed->format != MACRO_FORMAT ||
        flashed->rec_len != sizeof(MacroRecord) || flashed->count > MACRO_COUNT) {
        table_defaults(g_table);
        g_loaded = true;
        printf("[Macro] no valid table, starting empty\n");
        return;
    }

    memcpy(&g_table, flashed, sizeof(MacroTable));
    if (g_table.crc32 != table_crc(g_table)) {
        printf("[Macro] table CRC mismatch, starting empty\n");
        table_defaults(g_table);
    }
    g_loaded = true;
}

// Runs with core1 parked and interrupts off, exactly like config_save_flash_op.
// Without the core1 park this races the audio core and corrupts audio.
static void macro_flash_op(void *param) {
    const uint8_t *img = static_cast<const uint8_t *>(param);
    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(MACRO_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(MACRO_FLASH_OFFSET, img, MACRO_IMAGE_BYTES);
    restore_interrupts(interrupts);
}

bool macro_commit() {
    if (!g_loaded) macro_load();
    g_table.magic   = MACRO_MAGIC;
    g_table.format  = MACRO_FORMAT;
    g_table.count   = MACRO_COUNT;
    g_table.rec_len = sizeof(MacroRecord);
    g_table.crc32   = table_crc(g_table);

    alignas(4) static uint8_t image[MACRO_IMAGE_BYTES];
    memset(image, 0xff, sizeof(image));
    memcpy(image, &g_table, sizeof(MacroTable));

    const int rc = flash_safe_execute(macro_flash_op, image, 1000);
    if (rc != PICO_OK) {
        printf("[Macro] commit flash_safe_execute failed: %d\n", rc);
        return false;
    }
    const auto *v = reinterpret_cast<const MacroTable *>(XIP_BASE + MACRO_FLASH_OFFSET);
    if (v->crc32 != g_table.crc32) {
        printf("[Macro] commit verify failed\n");
        return false;
    }
    return true;
}

bool macro_get(uint8_t idx, MacroRecord &out) {
    if (idx >= MACRO_COUNT) return false;
    if (!g_loaded) macro_load();
    out = g_table.rec[idx];
    return true;
}

bool macro_set_entry(uint8_t idx, const MacroRecord &rec) {
    if (idx >= MACRO_COUNT) return false;
    if (!g_loaded) macro_load();
    g_table.rec[idx] = rec;
    // Editing an entry invalidates any chord armed against the old definition.
    g_chord = 0;
    return true;
}

// ============================== playback ===================================

// Send the keyboard state for a set of currently-pressed key SLOTS. Modifiers
// are ordinary usages 0xE0-0xE7 in keys[]; they fold into the modifier byte
// here, which is why the format needs no separate modifier field and why press
// order between a modifier and a key is unambiguous.
static bool send_state(uint32_t pressed_slots) {
    if (!tud_hid_n_ready(MACRO_KBD_INSTANCE)) return false;
    uint8_t mods = 0;
    uint8_t keys[6] = {0};
    uint8_t kn = 0;
    for (uint8_t i = 0; i < g_pb_n; i++) {
        if (!(pressed_slots & (1u << i))) continue;
        const uint8_t u = g_pb_keys[i];
        if (u >= HID_USAGE_MOD_FIRST && u <= HID_USAGE_MOD_LAST) {
            mods |= (uint8_t) (1u << (u - HID_USAGE_MOD_FIRST));
        } else if (u != 0 && kn < 6) {
            keys[kn++] = u;
        }
    }
    return tud_hid_n_keyboard_report(MACRO_KBD_INSTANCE, 0, mods, kn ? keys : nullptr);
}

// Which slot is released at release-position p.
static int slot_at_release_pos(uint8_t p) {
    for (uint8_t i = 0; i < g_pb_n; i++) {
        if (((g_pb_rel >> (2u * i)) & 3u) == p) return i;
    }
    return -1;
}

static uint32_t pressed_for_step(uint8_t step) {
    const uint32_t all = (g_pb_n >= 32) ? 0xFFFFFFFFu : ((1u << g_pb_n) - 1u);
    // Belt and braces: whatever the permutation says, the LAST step releases
    // everything. The guard in playback_start should make this unreachable, but
    // a stuck modifier at the host is bad enough to warrant two independent
    // defences rather than one.
    if (step >= (uint8_t) (2u * g_pb_n - 1u)) return 0;
    if (step < g_pb_n) {
        // Press phase: keys 0..step are down.
        return (step >= 31) ? all : ((1u << (step + 1)) - 1u);
    }
    // Release phase: positions 0..(step - n) have been released.
    uint32_t pressed = all;
    for (uint8_t p = 0; p <= (uint8_t) (step - g_pb_n); p++) {
        const int s = slot_at_release_pos(p);
        if (s >= 0) pressed &= ~(1u << s);
    }
    return pressed;
}

// rel_order must be a genuine permutation of 0..n-1: every release position
// claimed exactly once. If it is not, slot_at_release_pos() returns -1 for the
// unclaimed positions and those keys are NEVER released - playback ends with
// them still down and the host autorepeats them forever. rel_order == 0 is the
// default in a zeroed table and in anything a portal sends without setting it,
// and with n > 1 it makes every slot claim position 0, so the default value is
// itself the failure case. Verified on a host harness before this guard existed:
// Ctrl+J ended on [mods=00 keys=0D], i.e. J held down.
static bool rel_order_is_permutation(uint8_t rel, uint8_t n) {
    uint8_t seen = 0;
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t p = (uint8_t) ((rel >> (2u * i)) & 3u);
        if (p >= n) return false;
        if (seen & (uint8_t) (1u << p)) return false;
        seen |= (uint8_t) (1u << p);
    }
    return true;
}

// Reverse press order: last pressed is first released. The correct answer for
// essentially every real combo, and the only sane fallback.
static uint8_t reverse_press_order(uint8_t n) {
    uint8_t rel = 0;
    for (uint8_t i = 0; i < n; i++) {
        rel |= (uint8_t) ((uint8_t) (n - 1u - i) << (2u * i));
    }
    return rel;
}

static void playback_start(const MacroEntry &e) {
    g_pb_n = 0;
    for (uint8_t i = 0; i < MACRO_KEYS; i++) {
        if (e.keys[i] == 0) break;
        g_pb_keys[g_pb_n++] = e.keys[i];
    }
    if (g_pb_n == 0) return;
    g_pb_rel  = rel_order_is_permutation(e.rel_order, g_pb_n)
                    ? e.rel_order
                    : reverse_press_order(g_pb_n);
    g_pb_step = 0;
    g_pb_next = now_ms();
    g_pb_busy = true;
}

static void playback_task() {
    if (!g_pb_busy) return;
    const uint32_t now = now_ms();
    if ((int32_t) (now - g_pb_next) < 0) return;

    // If the endpoint is not ready, retry next pass rather than dropping a
    // transition - a lost release would leave a modifier latched at the host.
    if (!send_state(pressed_for_step(g_pb_step))) return;

    g_pb_step++;
    g_pb_next = now + MACRO_STEP_MS;
    if (g_pb_step >= (uint8_t) (2u * g_pb_n)) {
        g_pb_busy = false;
        g_pb_n    = 0;
    }
}

// ============================== matching ===================================

static inline uint8_t popcount32(uint32_t v) {
    uint8_t n = 0;
    while (v) { v &= (v - 1u); n++; }
    return n;
}

static int find_entry(uint32_t chord, bool want_long, uint32_t disable) {
    for (uint8_t i = 0; i < MACRO_COUNT; i++) {
        const MacroEntry &e = g_table.rec[i].entry;
        if (e.gesture != GESTURE_NONE || e.chord != chord || e.chord == 0) continue;
        if (!macro_is_enabled(disable, i)) continue;
        if (((e.flags & MACRO_FLAG_LONG_PRESS) != 0) != want_long) continue;
        return i;
    }
    return -1;
}

// Longest bound chord that is FULLY HELD wins. Note what this does and does not
// give you: a chord only matches once every one of its buttons is down, so with
// only R3+Up bound, holding R3 alone matches nothing and the macro fires the
// instant Up lands, however long R3 was held first.
//
// It does NOT suppress a bound SUBSET. Bind R3 as well and pressing R3 first
// arms and fires the R3 macro; the held branch then returns early and never
// re-evaluates, so R3+Up becomes unreachable. That is inherent - at the moment
// R3 goes down the firmware cannot know whether you meant "R3" or "the first
// half of R3+Up". The portal warns when one bound chord contains another rather
// than the firmware guessing with an arm delay.
static uint32_t best_chord(uint32_t mask, uint32_t disable) {
    uint32_t best = 0;
    uint8_t  best_bits = 0;
    for (uint8_t i = 0; i < MACRO_COUNT; i++) {
        const MacroEntry &e = g_table.rec[i].entry;
        if (e.gesture != GESTURE_NONE || e.chord == 0) continue;
        if (!macro_is_enabled(disable, i)) continue;
        if ((mask & e.chord) != e.chord) continue;
        const uint8_t bits = popcount32(e.chord);
        if (bits > best_bits) { best_bits = bits; best = e.chord; }
    }
    return best;
}

static void fire(int idx) {
    if (idx < 0 || g_suspended || g_pb_busy) return;
    playback_start(g_table.rec[idx].entry);
}

static uint32_t hold_ms_of(int idx) {
    if (idx < 0) return (uint32_t) MACRO_HOLD_CS_DEFAULT * 10u;
    const uint8_t cs = g_table.rec[idx].entry.hold_cs;
    return (uint32_t) (cs ? cs : MACRO_HOLD_CS_DEFAULT) * 10u;
}

// ============================== debounce ===================================

static uint32_t debounce(uint32_t raw, uint32_t now) {
    uint32_t out = raw;
    for (uint8_t i = 0; i < 32; i++) {
        const uint32_t b = 1u << i;
        if (raw & b) { g_off_since[i] = 0; continue; }
        if (!(g_stable & b)) { g_off_since[i] = 0; continue; }
        if (g_off_since[i] == 0) g_off_since[i] = now ? now : 1u;
        if ((now - g_off_since[i]) < MACRO_DEBOUNCE_MS) out |= b;   // still holding
        else g_off_since[i] = 0;                                    // real release
    }
    g_stable = out;
    return out;
}

// ============================== gestures ===================================

static void gesture_fire(uint8_t gesture, uint32_t disable) {
    for (uint8_t i = 0; i < MACRO_COUNT; i++) {
        const MacroEntry &e = g_table.rec[i].entry;
        if (e.gesture == GESTURE_NONE || e.gesture != gesture) continue;
        if (!macro_is_enabled(disable, i)) continue;
        fire(i);
        return;
    }
}

static void touch_task(const uint8_t *r, uint16_t len, uint32_t now, uint32_t disable) {
    const TouchPoint f1 = touch_point(r, len, 0);
    const TouchPoint f2 = touch_point(r, len, 1);

    if (f1.down && !g_touch_down) {
        g_touch_down = true;
        g_touch_two  = f2.down;
        g_touch_x0   = f1.x;
        g_touch_y0   = f1.y;
        g_touch_lx   = f1.x;
        g_touch_ly   = f1.y;
        g_touch_t0   = now;
        return;
    }
    if (f1.down) {
        if (f2.down) g_touch_two = true;   // second finger may land late
        g_touch_lx = f1.x;                 // end point comes from here, not the lift
        g_touch_ly = f1.y;
        return;
    }
    if (!g_touch_down) return;

    g_touch_down = false;
    const uint32_t dt = now - g_touch_t0;
    if (dt < GEST_MIN_MS || dt > GEST_MAX_MS) return;

    // The lifted sample's coordinates are stale - the DualSense stops updating
    // X/Y once bit7 goes high, so the report that tells us the finger left
    // carries wherever it was some time earlier. Classify from the last sample
    // taken while the finger was still down, or every swipe reads as noise.
    const int32_t dx = (int32_t) g_touch_lx - (int32_t) g_touch_x0;
    const int32_t dy = (int32_t) g_touch_ly - (int32_t) g_touch_y0;
    const int32_t adx = dx < 0 ? -dx : dx;
    const int32_t ady = dy < 0 ? -dy : dy;

    uint8_t dir;
    if (adx >= GEST_MIN_DX && adx > ady)      dir = dx > 0 ? GEST_DIR_RIGHT : GEST_DIR_LEFT;
    else if (ady >= GEST_MIN_DY && ady > adx) dir = dy > 0 ? GEST_DIR_DOWN  : GEST_DIR_UP;
    else return;

    uint8_t g = (uint8_t) (GEST_VALID | dir);
    if (g_touch_x0 > (TOUCH_X_MAX / 2)) g |= GEST_ZONE_RIGHT;
    if (g_touch_two)                    g |= GEST_TWO_FINGER;
    gesture_fire(g, disable);
}

// ============================== public =====================================

void macro_on_input(const uint8_t *report, uint16_t len) {
    // A truncated report must never be decoded: touch_point() would report
    // "finger up" for a report that simply ended early, and touch_task() would
    // classify a gesture out of stale coordinates.
    if (report == nullptr || len < RPT_MIN_LEN) return;
    if (!g_loaded) macro_load();

    const uint32_t disable = get_config().macro_disable;
    if (!macro_any_enabled(disable)) { g_chord = 0; g_stable = 0; return; }

    // Every wake rule applies: nothing may transmit while the host is
    // suspended, and nothing may be left latched when it goes down.
    if (wake_host_is_suspended()) { macro_reset(); return; }

    const uint32_t now  = now_ms();
    const uint32_t mask = debounce(button_mask(report, len), now);

    touch_task(report, len, now, disable);

    if (g_chord != 0) {
        const int long_idx  = find_entry(g_chord, true,  disable);
        const int short_idx = find_entry(g_chord, false, disable);

        if ((mask & g_chord) == g_chord) {
            if (!g_chord_fired && long_idx >= 0 &&
                (now - g_chord_start) >= hold_ms_of(long_idx)) {
                fire(long_idx);
                g_chord_fired = true;
            }
            return;
        }
        // Released. A short macro sharing the chord with a long one can only
        // fire here, because "short" is not knowable until the release.
        if (!g_chord_fired && short_idx >= 0) fire(short_idx);
        g_chord = 0;
        return;
    }

    const uint32_t chord = best_chord(mask, disable);
    if (chord == 0) return;

    g_chord       = chord;
    g_chord_start = now;
    g_chord_fired = false;

    // With no long macro on this chord there is nothing to wait for, so fire on
    // PRESS. Waiting for the release would add avoidable latency.
    if (find_entry(chord, true, disable) < 0) {
        const int short_idx = find_entry(chord, false, disable);
        if (short_idx >= 0) { fire(short_idx); g_chord_fired = true; }
    }
}

void macro_task() {
    if (wake_host_is_suspended()) return;
    // The wake FSM owns the shared keyboard instance from the moment it asks for
    // a resume until its F15 keyup has gone out. That sequence straddles the
    // resume, so host_suspended is already false for most of it - and wake is
    // triggered BY a button press, so the user is holding buttons exactly then.
    // Interleaving a macro there corrupts both sequences.
    if (wake_owns_keyboard()) return;
    playback_task();

    // A long press must still fire when the chord is held perfectly still and
    // the pad stops sending fresh reports.
    if (g_chord == 0 || g_chord_fired) return;
    const uint32_t disable = get_config().macro_disable;
    const int long_idx = find_entry(g_chord, true, disable);
    if (long_idx < 0) return;
    if ((now_ms() - g_chord_start) >= hold_ms_of(long_idx)) {
        fire(long_idx);
        g_chord_fired = true;
    }
}

void macro_reset() {
    g_chord      = 0;
    g_stable     = 0;
    g_touch_down = false;
    memset(g_off_since, 0, sizeof(g_off_since));
    if (g_pb_busy || g_pb_n) {
        // Blank report: never leave a modifier latched at the host. This is the
        // keyboard-side twin of state_release_for_suspend().
        if (tud_hid_n_ready(MACRO_KBD_INSTANCE)) {
            tud_hid_n_keyboard_report(MACRO_KBD_INSTANCE, 0, 0, nullptr);
        }
        g_pb_busy = false;
        g_pb_n    = 0;
        g_pb_step = 0;
    }
}

void macro_suspend(bool on) {
    g_suspended = on;
    if (on) macro_reset();
}

bool macro_busy() { return g_pb_busy; }
