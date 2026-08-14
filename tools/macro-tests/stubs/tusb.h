#pragma once
#include <cstdint>
#include <cstdio>
struct FakeKbdReport { uint8_t mods; uint8_t keys[6]; };
extern FakeKbdReport g_sent[64]; extern int g_sent_n; extern bool g_ep_ready;
static inline bool tud_hid_n_ready(uint8_t){ return g_ep_ready; }
static inline bool tud_hid_n_keyboard_report(uint8_t,uint8_t,uint8_t mods,const uint8_t*keys){
  FakeKbdReport r{}; r.mods=mods; if(keys) for(int i=0;i<6;i++) r.keys[i]=keys[i];
  if(g_sent_n<64) { g_sent[g_sent_n++]=r; }
  return true; }
