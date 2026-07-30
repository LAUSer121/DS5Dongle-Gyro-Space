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
    uint8_t  last_remote_wakeup_en;  // did the host PERMIT remote wakeup?
    uint8_t  last_disconnect_ok;     // did bt_disconnect() have a live handle?
    uint8_t  last_wake_tud_ok;       // tud_remote_wakeup() return
    uint8_t  last_wake_host_suspended;
};
extern wake_diag_t g_wake_diag;
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
