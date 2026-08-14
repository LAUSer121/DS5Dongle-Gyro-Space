//
// macro-tests.cpp - host-side regression tests for src/macro.cpp.
//
// Compiles the REAL engine (see run-macro-tests.sh, which copies src/macro.cpp
// into stubs/ unmodified so its quoted includes resolve to the stubs) against
// ~60 lines of fakes for TinyUSB, flash and time, plus a 4 MB fake flash array
// so macro_load/macro_commit exercise the true storage path. The crc32 in
// stubs/utils.h is lifted verbatim from src/utils.h.
//
// This exists because the stuck-key bug below was invisible to code reading and
// obvious in one run.
//

#include <cstdio>
#include <cstring>
#include <cstdint>

#include "tusb.h"
#include "hardware/flash.h"
#include "macro.h"
#include "input_buttons.h"
#include "config.h"
#include "flash_map.h"
#include "wake.h"

// ---------------------------------------------------------------- stub state
FakeKbdReport g_sent[64];
int  g_sent_n = 0;
bool g_ep_ready = true;
unsigned char g_fake_flash[PICO_FLASH_SIZE_BYTES];
uint32_t g_now_ms = 1000;

static Config_body g_cfg;
Config_body &get_config() { return g_cfg; }

static bool g_suspended_stub = false;
static bool g_wake_owns_stub = false;
bool wake_host_is_suspended(void) { return g_suspended_stub; }
bool wake_owns_keyboard(void)     { return g_wake_owns_stub; }

// ---------------------------------------------------------------- harness
static int g_fail = 0;
static void ok(bool cond, const char *what) {
    printf("  %s  %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) g_fail++;
}

static uint8_t rpt[64];
static void reset_report() {
    memset(rpt, 0, sizeof(rpt));
    rpt[RPT_BTN0] = 0x08;  // hat idle
    rpt[32] = 0x80;        // finger 1 up
    rpt[36] = 0x80;        // finger 2 up
}
static void btn_r3(bool on)  { if (on) rpt[RPT_BTN1] |= 0x80; else rpt[RPT_BTN1] &= ~0x80; }
static void btn_up(bool on)  { rpt[RPT_BTN0] = (uint8_t)((rpt[RPT_BTN0] & 0xF0) | (on ? 0x00 : 0x08)); }
static void touch(bool down, uint16_t x, uint16_t y) {
    rpt[32] = down ? 0x00 : 0x80;
    rpt[33] = (uint8_t)(x & 0xFF);
    rpt[34] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
    rpt[35] = (uint8_t)(y >> 4);
}
static void step(uint32_t dt) { g_now_ms += dt; macro_on_input(rpt, sizeof(rpt)); macro_task(); }
static void settle()          { for (int i = 0; i < 12; i++) step(20); }

static bool last_report_blank() {
    if (g_sent_n == 0) return true;
    const FakeKbdReport &r = g_sent[g_sent_n - 1];
    if (r.mods) return false;
    for (int k = 0; k < 6; k++) if (r.keys[k]) return false;
    return true;
}
static bool saw_key(uint8_t mods, uint8_t key) {
    for (int i = 0; i < g_sent_n; i++) {
        if (g_sent[i].mods != mods) continue;
        for (int k = 0; k < 6; k++) if (g_sent[i].keys[k] == key) return true;
    }
    return false;
}

static void fresh_device() {
    memset(g_fake_flash, 0xFF, sizeof(g_fake_flash));
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.macro_disable = MACRO_NONE_ENABLED;
    g_suspended_stub = false;
    g_wake_owns_stub = false;
    g_sent_n = 0;
    g_ep_ready = true;
    reset_report();
    macro_load();
    macro_reset();
    g_sent_n = 0;
}

static MacroRecord chord_macro(uint32_t chord, uint8_t k0, uint8_t k1,
                               uint8_t rel, bool longpress, uint8_t hold_cs = 0) {
    MacroRecord r{};
    r.entry.chord     = chord;
    r.entry.gesture   = GESTURE_NONE;
    r.entry.flags     = longpress ? MACRO_FLAG_LONG_PRESS : 0;
    r.entry.hold_cs   = hold_cs;
    r.entry.keys[0]   = k0;
    r.entry.keys[1]   = k1;
    r.entry.rel_order = rel;
    return r;
}
static void enable_only(uint32_t bits) { g_cfg.macro_disable = ~bits; }

constexpr uint8_t K_CTRL = 0xE0;
constexpr uint8_t K_ALT  = 0xE2;
constexpr uint8_t K_J    = 0x0D;
constexpr uint8_t K_TAB  = 0x2B;

// ---------------------------------------------------------------- tests

static void t_press_then_second_button() {
    printf("chord fires only when FULLY held, however long the first button waits\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3 | BTN_DPAD_UP, K_CTRL, K_J, 0, false));
    enable_only(1u);

    btn_r3(true);  step(10);
    step(200); step(200);
    ok(g_sent_n == 0, "R3 held 410ms alone fires nothing");

    btn_up(true);  step(10);
    ok(g_sent_n > 0, "fires the instant Up lands");
    settle();
    ok(saw_key(0x01, K_J), "Ctrl+J appears in the walk");
    ok(last_report_blank(), "walk ends on a blank report");
}

static void t_default_rel_order_no_stuck_key() {
    printf("rel_order = 0 (the default) must not strand a key\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3 | BTN_DPAD_UP, K_CTRL, K_J, 0, false));
    enable_only(1u);
    btn_r3(true); btn_up(true); step(10); settle();
    ok(last_report_blank(), "no key left down (the pre-fix bug ended on keys=0D)");
}

static void t_alt_tab_release_order() {
    printf("captured release order is honoured: Alt+Tab lifts Tab first\n");
    fresh_device();
    // slot0 = Alt released at position 1, slot1 = Tab released at position 0
    const uint8_t rel = (uint8_t)((1u << 0) | (0u << 2));
    macro_set_entry(0, chord_macro(BTN_R3, K_ALT, K_TAB, rel, false));
    enable_only(1u);
    btn_r3(true); step(10); settle();

    int i_alt_only_after_tab = -1;
    for (int i = 1; i < g_sent_n; i++) {
        if (g_sent[i].mods == 0x04 && g_sent[i].keys[0] == 0) { i_alt_only_after_tab = i; break; }
    }
    ok(saw_key(0x04, K_TAB), "Alt+Tab asserted");
    ok(i_alt_only_after_tab > 0, "Tab released while Alt still held");
    ok(last_report_blank(), "ends blank");
}

static void t_long_vs_short() {
    printf("short and long on one chord\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    macro_set_entry(1, chord_macro(BTN_R3, K_ALT, K_TAB, 0, true, 75));
    enable_only(0x3u);

    btn_r3(true); step(10);
    ok(g_sent_n == 0, "short does NOT fire on press when a long shares the chord");
    step(400);
    ok(g_sent_n == 0, "still nothing at 410ms");
    step(400);
    ok(g_sent_n > 0, "long fires past 750ms");
    settle();   // the walk needs MACRO_STEP_MS between reports; TAB lands on step 1
    ok(saw_key(0x04, K_TAB), "and the combo reaches Alt+Tab");
    g_sent_n = 0;
    btn_r3(false); step(10); settle();
    ok(!saw_key(0x01, K_J), "short suppressed on release after long fired");

    // and the short path when the press is brief
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    macro_set_entry(1, chord_macro(BTN_R3, K_ALT, K_TAB, 0, true, 75));
    enable_only(0x3u);
    btn_r3(true); step(10); step(100);
    btn_r3(false); step(10); step(60); settle();
    ok(saw_key(0x01, K_J), "brief press fires the short macro on release");
}

static void t_only_short_fires_on_press() {
    printf("no long macro on the chord -> fire on press, not release\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    enable_only(1u);
    btn_r3(true); step(10);
    ok(g_sent_n > 0, "fires immediately on press");
}

static void t_disabled_mask() {
    printf("the per-slot mask actually gates firing\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    g_cfg.macro_disable = MACRO_NONE_ENABLED;   // all off
    btn_r3(true); step(10); settle();
    ok(g_sent_n == 0, "nothing fires with all macros disabled");
    ok(!macro_any_enabled(g_cfg.macro_disable), "macro_any_enabled false -> no kbd interface");
}

static void t_suspend_and_wake_arbitration() {
    printf("wake rules\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    enable_only(1u);

    g_suspended_stub = true;
    btn_r3(true); step(10); settle();
    ok(g_sent_n == 0, "nothing transmits while the host is suspended");

    g_suspended_stub = false; btn_r3(false); step(10); settle(); g_sent_n = 0;
    g_wake_owns_stub = true;
    btn_r3(true); step(10);
    const int during = g_sent_n;
    settle();
    ok(during == g_sent_n, "playback stalls while the wake FSM owns the keyboard");
    g_wake_owns_stub = false;
    settle();
    ok(g_sent_n > during, "resumes once wake releases it");
    ok(last_report_blank(), "and still ends blank");
}

static void t_truncated_report() {
    printf("a truncated report is not decoded\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));
    enable_only(1u);
    btn_r3(true);
    macro_on_input(rpt, 8);      // shorter than RPT_MIN_LEN
    macro_task();
    ok(g_sent_n == 0, "short report ignored rather than half-decoded");
}

static void t_gesture() {
    printf("touchpad swipe\n");
    fresh_device();
    MacroRecord g{};
    g.entry.gesture   = (uint8_t)(GEST_VALID | GEST_DIR_RIGHT);
    g.entry.keys[0]   = K_CTRL;
    g.entry.keys[1]   = K_J;
    g.entry.rel_order = 0;
    macro_set_entry(0, g);
    enable_only(1u);

    touch(true, 200, 500);  step(10);
    touch(true, 700, 520);  step(60);
    touch(false, 0, 0);     step(10);       // lift: coords go stale, engine must use last-down
    settle();
    ok(saw_key(0x01, K_J), "left-to-right swipe fires its macro");
    ok(last_report_blank(), "gesture playback ends blank");
}

static void t_persistence() {
    printf("flash round trip\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3 | BTN_DPAD_UP, K_CTRL, K_J, 0, false));
    ok(macro_commit(), "commit succeeds and verifies");
    macro_load();
    MacroRecord back{};
    ok(macro_get(0, back), "entry reads back");
    ok(back.entry.chord == (BTN_R3 | BTN_DPAD_UP) && back.entry.keys[1] == K_J,
       "contents survive the round trip");

    // corrupt one record, CRC must reject the whole table
    g_fake_flash[MACRO_FLASH_OFFSET + sizeof(uint32_t) * 2 + 4] ^= 0xFF;
    macro_load();
    macro_get(0, back);
    ok(back.entry.chord == 0, "a corrupted table falls back to empty, not garbage");
}

static void t_virgin_flash() {
    printf("upgrade from a device that has never had macros\n");
    memset(g_fake_flash, 0xFF, sizeof(g_fake_flash));
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.macro_disable = MACRO_NONE_ENABLED;
    macro_load();
    MacroRecord r{};
    ok(macro_get(0, r) && r.entry.chord == 0 && r.entry.keys[0] == 0,
       "virgin sector yields an empty table, no migration needed");
}

static void t_known_subset_behaviour() {
    printf("KNOWN BEHAVIOUR: a bound subset shadows the longer chord\n");
    fresh_device();
    macro_set_entry(0, chord_macro(BTN_R3, K_CTRL, K_J, 0, false));               // subset
    macro_set_entry(1, chord_macro(BTN_R3 | BTN_DPAD_UP, K_ALT, K_TAB, 0, false));
    enable_only(0x3u);
    btn_r3(true); step(10); settle();
    btn_up(true); step(10); settle();
    ok(saw_key(0x01, K_J),   "R3 fired (pressed first)");
    ok(!saw_key(0x04, K_TAB),
       "R3+Up did NOT fire - documented, portal warns instead of an arm delay");
}

int main() {
    printf("=== macro engine tests ===\n");
    t_press_then_second_button();
    t_default_rel_order_no_stuck_key();
    t_alt_tab_release_order();
    t_long_vs_short();
    t_only_short_fires_on_press();
    t_disabled_mask();
    t_suspend_and_wake_arbitration();
    t_truncated_report();
    t_gesture();
    t_persistence();
    t_virgin_flash();
    t_known_subset_behaviour();
    printf("\n%s (%d failure%s)\n", g_fail ? "MACRO TESTS FAILED" : "MACRO TESTS OK",
           g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
