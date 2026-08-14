#pragma once
#include <cstdint>
extern uint32_t g_now_ms;
static inline uint32_t get_absolute_time(){ return g_now_ms; }
static inline uint32_t to_ms_since_boot(uint32_t t){ return t; }
