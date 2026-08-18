// Host test for the two-stage trigger output rewrite. Pulls the REAL code out of
// main.cpp so the maths under test is the maths that ships.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "cfg.inc"
static Config_body g_cfg;
static const Config_body &get_config() { return g_cfg; }
#include "t2.inc"

static int fails = 0;
static void ok(bool c, const char *m) { printf(c ? "  ok    %s\n" : "  FAIL  %s\n", m); if(!c) fails++; }

// Build a report with a given R2 axis value and the R2 digital bit set.
// Both axes, for the cross-trigger cases.
static void rpt2(uint8_t *r, uint8_t r2, uint8_t l2) {
    memset(r, 0, 63);
    r[7] = 0x08; r[5] = r2; r[4] = l2;
    if (r2 > 0) r[8] |= 0x08;
    if (l2 > 0) r[8] |= 0x04;
}

static void rpt(uint8_t *r, uint8_t r2) {
    memset(r, 0, 63);
    r[5] = r2;
    if (r2 > 0) r[8] |= 0x08;
}

// A trigger as the stage-2 target has to move the AXIS, not just the click bit
// in byte 8 - games read L2/R2 as analog axes and the digital bits are barely
// used, so setting the bit alone did nothing visible. It also has to survive the
// other trigger's own pass, which clears that same mask in its dead-zone branch.
static void t_trigger_target() {
    printf("\n-- trigger as the second-stage target --\n");
    uint8_t r[63];
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_L2;

    rpt2(r, 210, 0); apply_trigger_output(r);
    ok((r[8] & 0x04) && r[4] == 255, "R2 past the detent drives the L2 axis to full");
    rpt2(r, 150, 0); apply_trigger_output(r);
    ok(!(r[8] & 0x04) && r[4] == 0, "below the detent L2 is untouched");
    rpt2(r, 210, 120); apply_trigger_output(r);
    ok(r[4] == 255, "the synthetic press takes the max, never lowers a real pull");

    g_cfg.t2_l2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_l2_pos = 200; g_cfg.t2_l2_button = T2BTN_CROSS;
    rpt2(r, 210, 0); apply_trigger_output(r);
    ok(!(r[7] & 0x20), "R2 pressing L2 does not trip L2's own second stage");
    rpt2(r, 0, 210); apply_trigger_output(r);
    ok((r[7] & 0x20), "a real L2 pull still fires L2's own second stage");

    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_L2;
    g_cfg.at_l2_deadzone = 3;
    rpt2(r, 210, 0); apply_trigger_output(r);
    ok((r[8] & 0x04) && r[4] == 255, "an L2 dead zone does not clear R2's stage-2 press");
}

int main() {
    printf("=== two-stage trigger tests ===\n");
    uint8_t r[63];

    // --- off by default: the report must be untouched -----------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    rpt(r, 200); apply_trigger_output(r);
    ok(r[5] == 200 && r[7] == 0 && (r[8] & 0x08), "off: axis, buttons and digital bit untouched");
    ok(!trigger_output_active(g_cfg), "off: fast path stays available");

    // --- additive: axis untouched, button added above the boundary ----------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_CIRCLE;
    ok(trigger_output_active(g_cfg), "additive: gate reports active");
    rpt(r, 199); apply_trigger_output(r);
    ok(r[5] == 199 && r[7] == 0, "additive: below the boundary nothing changes");
    rpt(r, 200); apply_trigger_output(r);
    ok(r[5] == 200 && (r[7] & 0x40), "additive: at the boundary Circle is pressed");
    ok(r[8] & 0x08, "additive: the trigger stays held (racing keeps throttle)");

    // --- rescale: full 0-255 below the boundary -----------------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_RESCALE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_CROSS;
    rpt(r, 0);   apply_trigger_output(r); ok(r[5] == 0,   "rescale: 0 maps to 0");
    rpt(r, 100); apply_trigger_output(r); ok(r[5] == 127, "rescale: half travel maps to ~127");
    rpt(r, 199); apply_trigger_output(r);
    ok(r[5] == 253 && r[7] == 0, "rescale: just below the boundary is near full, no button");
    rpt(r, 200); apply_trigger_output(r);
    ok(r[5] == 255 && (r[7] & 0x20), "rescale: at the boundary the axis saturates and Cross fires");
    rpt(r, 255); apply_trigger_output(r);
    ok(r[5] == 255, "rescale: above the boundary the axis stays pinned, never wraps");

    // --- release stage 1: alt-fire --------------------------------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE | T2_RELEASE_STAGE1;
    g_cfg.t2_pos = 180; g_cfg.t2_button = T2BTN_R1;
    rpt(r, 179); apply_trigger_output(r);
    ok((r[8] & 0x08) && !(r[8] & 0x02), "release: below the boundary primary fire is held");
    rpt(r, 200); apply_trigger_output(r);
    ok(!(r[8] & 0x08), "release: above the boundary primary fire STOPS");
    ok(r[8] & 0x02, "release: and R1 takes over");

    // --- composition with the existing dead zone ------------------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.at_deadzone = 2;                       // zone 2 -> raw 51
    g_cfg.t2_mode = T2_AXIS_RESCALE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 40); apply_trigger_output(r);
    ok(r[5] == 0 && !(r[8] & 0x08), "dead zone still zeroes the axis and the digital bit");
    rpt(r, 51); apply_trigger_output(r);
    ok(r[5] == 0, "rescale starts from the dead zone edge, not from zero");
    rpt(r, 200); apply_trigger_output(r);
    ok(r[5] == 255 && (r[7] & 0x40), "rescale still reaches full at the boundary with a dead zone");

    // --- dead zone alone must behave exactly as before -------------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.at_deadzone = 2;
    rpt(r, 40); apply_trigger_output(r);
    ok(r[5] == 0 && !(r[8] & 0x08), "dead zone alone: unchanged legacy behaviour");
    rpt(r, 60); apply_trigger_output(r);
    ok(r[5] == 60 && (r[8] & 0x08), "dead zone alone: above the zone the axis is untouched");

    // --- pos 0 disables, and L2 is independent ---------------------------------
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_RESCALE; g_cfg.t2_pos = 0; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 255); apply_trigger_output(r);
    ok(r[5] == 255 && r[7] == 0, "boundary 0 disables the stage");

    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_l2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_l2_pos = 100; g_cfg.t2_l2_button = T2BTN_L1;
    memset(r, 0, 63); r[4] = 120; r[5] = 255; r[8] |= 0x04 | 0x08;
    apply_trigger_output(r);
    ok((r[8] & 0x01) && r[5] == 255, "L2 stage fires without disturbing R2");


    // --- release hysteresis --------------------------------------------------
    // Resting on the boundary made the button press and release at report rate;
    // a game reacting to it (nitro engaging over and over) reads as a buzz at
    // the wall. Engage at pos, release at pos - T2_HYST.
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 200); apply_trigger_output(r); ok(r[7] & 0x40, "hysteresis: engages at the boundary");
    rpt(r, 196); apply_trigger_output(r); ok(r[7] & 0x40, "hysteresis: stays engaged just below it");
    rpt(r, 193); apply_trigger_output(r); ok(r[7] & 0x40, "hysteresis: still held one count above release");
    rpt(r, 191); apply_trigger_output(r); ok(!(r[7] & 0x40), "hysteresis: releases below pos - 8");
    rpt(r, 196); apply_trigger_output(r); ok(!(r[7] & 0x40), "hysteresis: does NOT re-engage until the boundary");
    rpt(r, 200); apply_trigger_output(r); ok(r[7] & 0x40, "hysteresis: re-engages at the boundary");

    // Dither on the boundary must produce ONE press, not one per report.
    rpt(r, 0); apply_trigger_output(r);                       // clear the latch
    { int edges = 0; bool prev = false;
      const uint8_t dither[] = {199,200,199,201,200,198,201,199,200,202};
      for (unsigned k = 0; k < sizeof(dither); k++) {
          rpt(r, dither[k]); apply_trigger_output(r);
          const bool on = (r[7] & 0x40) != 0;
          if (on && !prev) edges++;
          prev = on;
      }
      ok(edges == 1, "hysteresis: dithering across the boundary gives ONE press, not a stream");
    }

    // A boundary at or below T2_HYST must still be able to release.
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 5; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 10); apply_trigger_output(r); ok(r[7] & 0x40, "low boundary: engages");
    rpt(r, 0);  apply_trigger_output(r); ok(!(r[7] & 0x40), "low boundary: still releases at rest");

    // Turning the feature off must drop a held latch, not strand it.
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 200; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 255); apply_trigger_output(r); ok(r[7] & 0x40, "latch set before disabling");
    g_cfg.t2_mode = T2_AXIS_OFF;
    rpt(r, 255); apply_trigger_output(r); ok(!(r[7] & 0x40), "disabling clears the latch");
    g_cfg.t2_mode = T2_AXIS_ADDITIVE;
    rpt(r, 195); apply_trigger_output(r);
    ok(!(r[7] & 0x40), "and re-enabling needs a fresh cross of the boundary");

    // A slot activation can retune the boundary while the trigger is held. The
    // latch must not survive that: it was set against a boundary that no longer
    // exists. Caught by test isolation, which is a real field case.
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.t2_mode = T2_AXIS_ADDITIVE; g_cfg.t2_pos = 100; g_cfg.t2_button = T2BTN_CIRCLE;
    rpt(r, 255); apply_trigger_output(r); ok(r[7] & 0x40, "retune: latched at the old boundary");
    g_cfg.t2_pos = 240;
    rpt(r, 200); apply_trigger_output(r);
    ok(!(r[7] & 0x40), "retune: a new boundary drops the stale latch");

    t_trigger_target();

    printf(fails ? "\nT2 TESTS FAILED (%d)\n" : "\nT2 TESTS OK (0 failures)\n", fails);
    return fails ? 1 : 0;
}
