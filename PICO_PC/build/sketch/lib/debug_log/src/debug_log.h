#line 1 "C:\\Users\\franc\\Desktop\\isaac\\def_CAN_TX_RX\\can_exchange\\lib\\debug_log\\src\\debug_log.h"
#pragma once
#include <stdint.h>
#include <stddef.h>

// ===================== CONFIG =====================
#define LOG_SIZE 4096

#ifdef __cplusplus
extern "C" {
#endif

// ===================== BUFFER =====================
extern char debug_log[LOG_SIZE];
extern volatile size_t log_head;

// ===================== FUNZIONI =====================
void dbg(const char* msg);
void dbg_int(int value);
void dbg_hex(uint32_t v);
void dbg_float(float value);

#ifdef __cplusplus
}
#endif
