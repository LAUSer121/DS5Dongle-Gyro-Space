//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "button_functions.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "wake.h"
#ifdef ENABLE_WAKE_HID
#include "ps_shortcut.h"
#endif
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "state_mgr.h"
#if ENABLE_SERIAL
#include "pico/stdio_usb.h"
#endif
#include "config.h"
#include "cmd.h"
#include "dse.h"
#include "gyro_fusion.h"
#include "gyro_space.h"
#include "hardware/timer.h"
#if ENABLE_BATT_LED
#include "battery_led.h"
#endif

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

int reportSeqCounter = 0;
uint8_t packetCounter = 0;
bool spk_active = false;

uint8_t interrupt_in_data[63] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

// Trigger activation dead zone (v1.8.0): mask what the HOST sees until the pull
// reaches the configured zone - analog forced to 0 and the digital press bit
// cleared, so games that fire on a hair-trigger register the action exactly where
// the resistance/detent/bow feel says they should. Applied ONLY to the outbound
// report copy: every internal consumer (AT gating, kick, shapes, gyro) keeps
// reading the raw trigger. Report body: [4]=L2 analog, [5]=R2 analog,
// [8] bit2=L2 pressed, bit3=R2 pressed. Zone N starts at N*25.5 counts.
static inline void apply_trigger_deadzone(uint8_t *r) {
    const auto &c = get_config();
    if (c.at_deadzone) {        // R2
        const uint8_t thr = (uint8_t)(((uint16_t)c.at_deadzone * 51u) / 2u);
        if (r[5] < thr) { r[5] = 0; r[8] &= (uint8_t)~0x08; }
    }
    if (c.at_l2_deadzone) {     // L2
        const uint8_t thr = (uint8_t)(((uint16_t)c.at_l2_deadzone * 51u) / 2u);
        if (r[4] < thr) { r[4] = 0; r[8] &= (uint8_t)~0x04; }
    }
}

void __not_in_flash_func(interrupt_loop)() {
    if (!tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        const auto &cdz = get_config();
        if (cdz.at_deadzone || cdz.at_l2_deadzone) {
            static uint8_t dz_report[63];
            memcpy(dz_report, interrupt_in_data, 63);
            apply_trigger_deadzone(dz_report);
            if (!tud_hid_report(0x01, dz_report, 63)) {
                printf("[USBHID] tud_hid_report error\n");
            }
        } else if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send. 
    uint8_t safe_report[63];


    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        apply_trigger_deadzone(safe_report); // no-op when both dead zones are 0
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");

            // If the report failed to queue, restore the dirty flag 
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

// --- Gyro aiming space (v1.19.0) -------------------------------------------
// Steam-Input-style pipeline. The old fixed-horizon mapping is gone:
//
//   gyro + accelerometer
//        -> sensor fusion (Mahony complementary AHRS)
//        -> quaternion orientation estimation
//        -> orientation space conversion (YAW/ROLL/YAW_ROLL/LOCAL_SPACE/
//           PLAYER_SPACE/WORLD_SPACE/LASER_POINTER)
//        -> unified gyro_x / gyro_y
//        -> right-stick output
//
// No fixed horizontal reference: the accelerometer's gravity vector aligns the
// orientation to the world in ANY grip. Quaternion normalization, gyro drift
// compensation and static calibration offsets live in the fusion module.
// All state is RAM/static; runs inside the report critical section.
volatile uint16_t g_diag_gyro = 0; // |gyro_x rate| diagnostic, field 0x35
// Live IMU diagnostics for the portal curves (fields 0x6a-0x6f), raw int16 LSB.
// Updated on every BT input report so the curves show real sensor data even
// while gyro aiming is off.
volatile int16_t g_diag_imu_gx = 0, g_diag_imu_gy = 0, g_diag_imu_gz = 0;
volatile int16_t g_diag_imu_ax = 0, g_diag_imu_ay = 0, g_diag_imu_az = 0;

static GyroFusion g_fusion;
static GyroSpace  g_space;
static bool       g_gyro_ready = false;
static uint64_t   g_gyro_last_us = 0;

static inline void __not_in_flash_func(apply_gyro_stick)(uint8_t *d) {
    auto rd16 = [&](int off) -> int32_t {
        return (int16_t)((uint16_t)d[off] | ((uint16_t)d[off + 1] << 8));
    };
    // Live IMU diagnostics for the portal curves (fields 0x6a-0x6f). Captured
    // even when gyro aiming is off so the curves always show real sensor data.
    { extern volatile int16_t g_diag_imu_gx, g_diag_imu_gy, g_diag_imu_gz,
             g_diag_imu_ax, g_diag_imu_ay, g_diag_imu_az;
      g_diag_imu_gx = (int16_t)rd16(15); g_diag_imu_gy = (int16_t)rd16(19); g_diag_imu_gz = (int16_t)rd16(17);
      g_diag_imu_ax = (int16_t)rd16(21); g_diag_imu_ay = (int16_t)rd16(23); g_diag_imu_az = (int16_t)rd16(25); }

    // Float accumulator for the stick output: at high report rates the per-frame
    // gyro increment is a fraction of one stick count, and rounding each frame
    // to an integer silently ate slow movements (the stick barely moved). The
    // fractional remainder carries over to the next frame instead.
    static float g_gyro_acc_x = 0.0f, g_gyro_acc_y = 0.0f;

    const auto &cfg = get_config();
    if (cfg.gyro_mode == 0) {
        g_gyro_ready = false;
        g_gyro_acc_x = g_gyro_acc_y = 0.0f;
        return;
    }
    // Activation schemes (industry set: ADS-gated, always-on, touch-enable,
    // ratchet, shoulder/fire gates):
    //   1 = only while L2 (aim) held past ~12%
    //   2 = always on
    //   3 = only while the touchpad is touched (Steam 'touch to enable' style)
    //   4 = always on, touching the touchpad PAUSES gyro (ratchet)
    //   5 = only while R2 (fire) held  6 = L1 held  7 = R1 held
    const bool touch = !(d[32] & 0x80);            // touchpad finger 1 down
    bool active = true;
    if (cfg.gyro_mode == 1 && d[4] < 30) active = false;              // L2
    if (cfg.gyro_mode == 3 && !touch)    active = false;
    if (cfg.gyro_mode == 4 && touch)     active = false;
    if (cfg.gyro_mode == 5 && d[5] < 30) active = false;              // R2
    if (cfg.gyro_mode == 6 && !(d[8] & 0x01)) active = false;         // L1
    if (cfg.gyro_mode == 7 && !(d[8] & 0x02)) active = false;         // R1

    // Hardware-verified IMU layout: AngularVelocityX(pitch)=15, Z(yaw)=17,
    // Y(roll)=19; AccelerometerX=21, Y=23, Z=25 (all int16 LE).
    float gyro[3] = {
        (float)rd16(15) * GYRO_DEG_PER_LSB,  // pitch rate (deg/s)
        (float)rd16(19) * GYRO_DEG_PER_LSB,  // roll rate (deg/s)
        (float)rd16(17) * GYRO_DEG_PER_LSB,  // yaw rate (deg/s)
    };
    const float accel[3] = {(float)rd16(21), (float)rd16(23), (float)rd16(25)};

    // Static calibration offsets (raw LSB) - unit-to-unit zero-offset trimming.
    gyro[0] -= (float)cfg.gyro_cal_x * GYRO_DEG_PER_LSB;
    gyro[1] -= (float)cfg.gyro_cal_y * GYRO_DEG_PER_LSB;
    gyro[2] -= (float)cfg.gyro_cal_z * GYRO_DEG_PER_LSB;

    // Timestep for the integration (clamped to sane bounds).
    const uint64_t now_us = time_us_64();
    float dt = (float)(now_us - g_gyro_last_us) * 1e-6f;
    g_gyro_last_us = now_us;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.05f)   dt = 0.05f;

    if (!g_gyro_ready) {
        gyro_fusion_init(&g_fusion, accel);
        gyro_space_init(&g_space, (GyroMode)cfg.gyro_space);
        g_gyro_ready = true;
    } else if ((uint8_t)g_space.mode != cfg.gyro_space) {
        // Mode changed live: re-arm the space (keeps fusion state).
        gyro_space_init(&g_space, (GyroMode)cfg.gyro_space);
    }

    // Sensor fusion: gyro + accelerometer -> orientation quaternion.
    gyro_fusion_update(&g_fusion, gyro, accel, dt, cfg.gyro_fusion);

    // PLAYER_SPACE reference capture on activation edge (Steam-style).
    gyro_space_tick(&g_space, active, g_fusion.q);

    // Orientation space conversion -> unified aim output (+x = right, +y = up).
    GyroOutput out;
    gyro_space_output(&g_space, g_fusion.q, gyro, &out);

    // Live diagnostic (portal, field 0x35): |aim-space horizontal rate| lets
    // sensitivity be calibrated against real numbers (0 = inactive).
    { extern volatile uint16_t g_diag_gyro;
      float h = out.x < 0.0f ? -out.x : out.x;
      g_diag_gyro = (h > 65535.0f) ? 65535 : (uint16_t)h; }

    if (!active) return;

    // Small deadzone against residual sensor noise at rest.
    if (out.x > -0.5f && out.x < 0.5f) out.x = 0.0f;
    if (out.y > -0.5f && out.y < 0.5f) out.y = 0.0f;

    // Per-report stick offset (no dt multiplier). The stick position directly
    // encodes camera angular velocity for the game — it is NOT a physical
    // displacement to be integrated over time. Matching artzox behaviour:
    //   dx = -raw_LSB * sens / 200      (where raw_LSB ≈ deg/s * 1638/100)
    //   dx ≈ -deg/s * sens * 0.082      (our equivalent using deg/s from fusion)
    //
    // sens=50, 100°/s yaw → out.x ≈ -100 → dx ≈ -410 → stick pegged (same as
    // artzox). Lower rates / lower sens produce proportionally smaller offsets.
    // Float accumulators preserve sub-count remainder for very slow (<1°/s)
    // movements so even tiny rotations produce a visible tick every few frames.
    const float s = cfg.gyro_sens * 0.082f;
    float dx = out.x * s;
    float dy = out.y * s;
    if (cfg.gyro_invert & 1) dx = -dx;
    if (cfg.gyro_invert & 2) dy = -dy;
    g_gyro_acc_x += dx;
    g_gyro_acc_y += dy;
    const int32_t ix = (int32_t)g_gyro_acc_x;
    const int32_t iy = (int32_t)g_gyro_acc_y;
    g_gyro_acc_x -= (float)ix;
    g_gyro_acc_y -= (float)iy;
    int32_t rx = (int32_t)d[2] + ix;
    int32_t ry = (int32_t)d[3] + iy;
    d[2] = (uint8_t)(rx < 0 ? 0 : (rx > 255 ? 255 : rx));
    d[3] = (uint8_t)(ry < 0 ? 0 : (ry > 255 ? 255 : ry));
}

void __not_in_flash_func(on_bt_data)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    if (channel == INTERRUPT && len > 2 && data[1] == 0x31) {
        // Mic audio: controller signals mic payload via bit1 of data[2];
        // the opus-encoded mic frame starts at data+4.
        if ((data[2] >> 1) & 1) {
            if (len >= 4) {
                mic_add_queue(data + 4, len - 4);
            }
            return;
        }
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }

        // Wake-on-PS must observe every BT input report regardless of polling
        // mode: the wake feature has its own state to maintain (button-byte
        // diff for edge detection) and short-circuiting it on non-2 polling
        // modes silently breaks wake while the host is suspended.
        wake_on_bt_input(data + 3, len - 3);
        #ifdef ENABLE_WAKE_HID
        ps_shortcut_tick(data + 3, len - 3);
        #endif

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
            apply_gyro_stick(interrupt_in_data);
            { extern volatile uint8_t g_l2_pos, g_r2_pos, g_l1_btn, g_r1_btn; g_l2_pos = interrupt_in_data[4]; g_r2_pos = interrupt_in_data[5]; g_l1_btn = (interrupt_in_data[8] & 0x01) ? 1 : 0; g_r1_btn = (interrupt_in_data[8] & 0x02) ? 1 : 0; } // L2@4 R2@5 L1/R1@8
#if ENABLE_BATT_LED
            battery_led_note_report();
#endif
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        apply_gyro_stick(interrupt_in_data);
        report_dirty = true;
        critical_section_exit(&report_cs);
        { extern volatile uint8_t g_l2_pos, g_r2_pos, g_l1_btn, g_r1_btn; g_l2_pos = data[3 + 4]; g_r2_pos = data[3 + 5]; g_l1_btn = (data[3 + 8] & 0x01) ? 1 : 0; g_r1_btn = (data[3 + 8] & 0x02) ? 1 : 0; } // L2@4 R2@5 L1/R1@8
#if ENABLE_BATT_LED
        battery_led_note_report();
#endif
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        if (reqlen >= 8) {
            memset(buffer, 0, 8);
            return 8;
        }
        return 0;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    // DSE profiles: while the unlock + prefetch is still in progress, return 0
    // (NAK) for profile reads so the PS app retries rather than caching an
    // empty snapshot. Still kick off the background BT fetch.
    if (dse_is_profile_report(report_id) && !dse_profiles_ready()) {
        get_feature_data(report_id, reqlen);
        return 0;
    }

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        // 0x81 (portal command replies) and 0x82 (slot-command replies, split off
        // to dodge the portal's 0x81 diagnostic poll) both carry a full 0x66-framed
        // reply that must be returned VERBATIM. Every other report id is a native
        // report whose stored leading byte is the report id and gets stripped.
        // CLAMP to reqlen in every path: TinyUSB sizes the transfer buffer from
        // the DESCRIPTOR-declared report size. Copying more than reqlen is a
        // buffer overflow in the USB stack (this is exactly how routing 63-byte
        // slot replies through 0x82 - declared as a 9-byte report - corrupted
        // reads and threw errors in WebHID). Slot replies live on 0x84, whose
        // declared size is the full 63 bytes.
        if ((report_id == 0x81 || report_id == 0x84) && feature_data[0] == 0x66) {
            const uint16_t n = (uint16_t)((feature_data.size() < reqlen) ? feature_data.size() : reqlen);
            memcpy(buffer, feature_data.data(), n);
            return n;
        }
        const uint16_t n = (uint16_t)(((feature_data.size() - 1) < reqlen) ? (feature_data.size() - 1) : reqlen);
        memcpy(buffer, feature_data.data() + 1, n);
        return n;
    }

    return 0;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex); // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue); // bAlternateSetting

    if (itf == 1) {
        printf("[AUDIO] Set interface Speaker to alternate setting %d\n", alt);
        spk_active = alt;
    }
    if (itf == 2) { // ITF_NUM_AUDIO_STREAMING_IN (microphone)
        printf("[AUDIO] Set interface Microphone to alternate setting %d\n", alt);
        set_mic_active(alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        // Drop keyboard SET_REPORT (host LED state).
        return;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                bool changed = state_update(buffer + 1, bufsize - 1);
                if (spk_active && !changed) {
                    break;
                }
                uint8_t outputData[78]{};
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) {
                    reportSeqCounter = 0;
                }
                outputData[2] = 0x10;
                // memcpy(outputData + 3, buffer + 1, bufsize - 1);
                state_set(outputData + 3, sizeof(SetStateData));
                bt_write(outputData, sizeof(outputData));
                break;
            }
        }
    }
    if (report_id == 0x80 && bufsize >= 2 && buffer[0] == 0x66) {
#if ENABLE_VERBOSE
        printf("[HID] Receive 0x66 setting config, funcid:0x%02X\n", buffer[1]);
#endif

        // 0x80 0x66 cmd_id payload...
        pico_cmd_set(buffer[1], buffer + 2, bufsize - 2);
        return;
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        set_feature_data(report_id, const_cast<uint8_t *>(buffer), bufsize);
    }
}

int main() {
#if SYS_CLOCK_KHZ != 150000
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
#endif

    board_init();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#if !ENABLE_SERIAL
    sleep_ms(150);
    tud_disconnect();
#endif
    board_init_after_tusb();
#if ENABLE_SERIAL
    stdio_usb_init();
#endif

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    // SMPS coil-whine fix: at light load the on-board buck regulator drops into PFM
    // (power-save) mode, and its pulse-skipping repetition rate falls into the
    // audible band -> the board whines at idle. Driving the CYW43 SMPS power-save
    // control pin (WL_GPIO1 on the Pico 2 W / Pico W) HIGH forces continuous PWM,
    // which has lower 3V3 ripple at light load and silences the whine. No-op on
    // boards without the pin. (From awalol PR #207, independent of Wake-on-LAN.)
#ifndef CYW43_WL_GPIO_SMPS_PIN
#define CYW43_WL_GPIO_SMPS_PIN 1   // WL_GPIO1 on Pico W / Pico 2 W
#endif
    cyw43_arch_gpio_put(CYW43_WL_GPIO_SMPS_PIN, true);

#if ENABLE_BATT_LED
    battery_led_init();
#endif

#if !ENABLE_SERIAL
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            } else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
#endif

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    wake_init();

    config_load();

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();
    state_init();

#if !ENABLE_SERIAL
    watchdog_enable(1000, true);
#endif

    while (1) {
#if !ENABLE_SERIAL
        watchdog_update();
#endif
        // Synth tick: with the host quiet, gated adaptive triggers must still
        // engage/release from live trigger movement, and releases must actually
        // reach the controller (fixes triggers stuck in resistance after rapid
        // R2/L2 play in games that only send reports when rumble changes).
        // Host just went to sleep: actively release the triggers ONCE before
        // standing down, so nothing stays latched on the controller through the
        // sleep and across the deferred power-off the wake path relies on.
        {
            static bool was_suspended = false;
            const bool susp = wake_host_is_suspended();
            if (susp && !was_suspended && state_release_for_suspend()) {
                uint8_t outputData[78]{};
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) reportSeqCounter = 0;
                outputData[2] = 0x10;
                state_set(outputData + 3, sizeof(SetStateData));
                bt_write(outputData, sizeof(outputData));
            }
            was_suspended = susp;
        }
        {
            static uint32_t last_synth_tick_ms = 0;
            const uint32_t now = to_ms_since_boot(get_absolute_time());
            // 8 ms, not 50: a custom-effect stage sequence latches on trigger
            // POSITION, and a pull takes ~100-200 ms, so a 50 ms cadence gave only
            // 2-4 samples per pull and routinely skipped a stage's arming window
            // ("sometimes I get the wall, sometimes I don't"). The call is cheap -
            // it early-returns unless the host has gone quiet, and only pushes a
            // report when the composed state actually changes.
            // While the host is SUSPENDED there is nothing to synthesize for, and
            // the extra BT output traffic competes with the input reports that
            // wake-on-PS has to observe - so stand down completely until resume.
            // (Raising this cadence from 50 ms without that guard is what made
            // wake less reliable than 1.13.3.)
            // Interval is re-evaluated EVERY pass (cheap), so trigger movement
            // restores the fast cadence immediately; only the tick is rate-limited.
            if (!wake_host_is_suspended() &&
                now - last_synth_tick_ms >= state_synth_interval_ms()) {
                last_synth_tick_ms = now;
                if (state_synth_tick()) {
                    uint8_t outputData[78]{};
                    outputData[0] = 0x31;
                    outputData[1] = reportSeqCounter << 4;
                    if (++reportSeqCounter == 256) reportSeqCounter = 0;
                    outputData[2] = 0x10;
                    state_set(outputData + 3, sizeof(SetStateData));
                    bt_write(outputData, sizeof(outputData));
                }
            }
        }
        cyw43_arch_poll();
        tud_task();
        wake_task();
        audio_loop();
        interrupt_loop();
#if ENABLE_BATT_LED
        battery_led_tick();
#endif
        button_check();
        bt_inquiring_led();
        dse_task();
    }
}
