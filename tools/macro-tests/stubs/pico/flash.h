#pragma once
#define PICO_OK 0
static inline int flash_safe_execute(void(*fn)(void*),void*p,uint32_t){ fn(p); return 0; }
