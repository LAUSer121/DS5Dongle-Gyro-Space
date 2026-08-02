//
// Created by awalol on 2026/4/30.
//

#ifndef DS5_BRIDGE_WAKE_H
#define DS5_BRIDGE_WAKE_H

#include <cstdint>

#ifdef ENABLE_WAKE_HID
void wake_init(void);
void wake_on_bt_connect(void);
void wake_on_bt_input(const uint8_t *hid_input, uint16_t len);
void wake_on_bt_disconnect(void);
void wake_task(void);
void wake_note_usb_reconnect(void);

// RAM-only counters so an intermittent wake failure can be inspected after the
// fact instead of guessed at. Read over HID with command 0x16.
struct wake_diag_t {
    uint16_t suspend_cb_count;       // times tud_suspend_cb actually fired
    uint16_t recovered_suspends;     // suspends caught only by polling tud_suspended()
    uint16_t recovered_resumes;      // resumes caught only by polling
    uint16_t disconnect_attempts;    // bt_disconnect() calls at suspend
    uint16_t wake_attempts;          // request_host_wake() calls
    uint16_t dcd_forced;             // times the DCD fallback was used
    uint16_t resume_reissues;        // resume pulses re-sent when the host didn't wake
    uint8_t  last_remote_wakeup_en;  // did the host PERMIT remote wakeup?
    uint8_t  last_disconnect_ok;     // did bt_disconnect() have a live handle?
    uint8_t  last_wake_tud_ok;       // tud_remote_wakeup() return
    uint8_t  last_wake_host_suspended;
};
extern wake_diag_t g_wake_diag;

// A snapshot of the MOST RECENT suspend->resume cycle. Cumulative counters can't
// tell you which sleep failed; this can.
struct wake_cycle_t {
    uint8_t requests;    // wake requests made while actually suspended
    uint8_t accepted;    // ...that the USB stack accepted
    uint8_t dcd;         // ...that fell back to driving the bus directly
    uint8_t reissues;    // resume pulses re-sent
    uint8_t resumed;     // did the bus come back?
    uint8_t key_sent;    // was the wake keypress delivered?
    uint8_t hid_waited;  // resumed but the keyboard endpoint was not ready
    uint8_t hid_timeout; // ...and never became ready
    uint8_t end_state;   // state machine state when the cycle ended
};
extern wake_cycle_t g_wake_cycle;
bool wake_host_is_suspended(void);
#else
static inline void wake_init(void) {}
static inline void wake_on_bt_connect(void) {}
static inline void wake_on_bt_input(const uint8_t *, uint16_t) {}
static inline void wake_on_bt_disconnect(void) {}
static inline void wake_task(void) {}
static inline void wake_note_usb_reconnect(void) {}
static inline bool wake_host_is_suspended(void) { return false; }
#endif

#endif //DS5_BRIDGE_WAKE_H
