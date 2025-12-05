#include "debug_log.h"

char debug_log[LOG_SIZE];
volatile size_t log_head = 0;

void dbg(const char* msg) {
    size_t i = 0;
    while (msg[i]) {
        debug_log[log_head] = msg[i++];
        log_head = (log_head + 1) % LOG_SIZE;
    }
    debug_log[log_head] = '\0';  // terminatore stringa
}

void dbg_int(int value) {
    char buf[16];
    int i = 0;
    if (value < 0) {
        debug_log[log_head++] = '-';
        value = -value;
    }
    do {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    } while (value > 0);

    while (i--) {
        debug_log[log_head++] = buf[i];
        if (log_head >= LOG_SIZE) log_head = 0;
    }
    debug_log[log_head] = '\0';
}

void dbg_hex(uint32_t v) {
    const char* hex = "0123456789ABCDEF";
    dbg("0x");
    for (int i = 7; i >= 0; i--) {
        debug_log[log_head++] = hex[(v >> (i*4)) & 0xF];
        if (log_head >= LOG_SIZE) log_head = 0;
    }
    debug_log[log_head] = '\0';
}

void dbg_float(float value) {
    int int_part = (int)value;
    int decimal = (int)((value - int_part) * 1000);
    dbg_int(int_part);
    dbg(".");
    if (decimal < 0) decimal = -decimal;
    dbg_int(decimal);
}
