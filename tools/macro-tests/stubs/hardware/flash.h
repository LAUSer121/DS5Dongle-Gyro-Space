#pragma once
#include <cstdint>
#include <cstring>
#define FLASH_SECTOR_SIZE 4096u
#define FLASH_PAGE_SIZE 256u
#define PICO_FLASH_SIZE_BYTES (4u*1024u*1024u)
extern unsigned char g_fake_flash[PICO_FLASH_SIZE_BYTES];
#define XIP_BASE ((uintptr_t)g_fake_flash)
static inline void flash_range_erase(uint32_t o,uint32_t n){ memset(g_fake_flash+o,0xFF,n); }
static inline void flash_range_program(uint32_t o,const uint8_t*d,uint32_t n){ memcpy(g_fake_flash+o,d,n); }
