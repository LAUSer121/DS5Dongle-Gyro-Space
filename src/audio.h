//
// Created by awalol on 2026/3/5.
//

#ifndef DS5_BRIDGE_AUDIO_H
#define DS5_BRIDGE_AUDIO_H

#include <cstdint>

void audio_init();
void audio_loop();
void core1_entry();
void set_headset(bool state);
void set_mic_active(bool active);
bool audio_mic_active();
void mic_add_queue(uint8_t *data, uint16_t len);
void update_mic_status();

// core1 / flash-safe victim health (see audio.cpp). state: 0 = not started,
// 1 = running, 2 = exited (Opus encoder could not be created) - in state 2 the
// core no longer answers flash lockouts, which is why config.cpp drops the
// registration before writing. heartbeat freezes when the core stops looping.
extern volatile uint8_t  g_core1_state;
extern volatile int32_t  g_core1_init_error;
extern volatile uint32_t g_core1_heartbeat;

#endif //DS5_BRIDGE_AUDIO_H