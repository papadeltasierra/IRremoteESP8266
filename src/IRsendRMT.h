// Copyright 2026 GitHub Copilot

#ifndef IRSENDRMT_H_
#define IRSENDRMT_H_

#include <stdint.h>

#ifdef ESP32
#include <driver/rmt.h>

void IRsendRMT_begin(uint16_t pin);
void IRsendRMT_enableIROut(uint16_t pin, uint32_t freq, uint8_t duty,
                           bool modulation, bool active_high);
uint16_t IRsendRMT_mark(uint16_t pin, uint16_t usec, bool modulation,
                        bool active_high, uint32_t period);
void IRsendRMT_space(uint16_t pin, uint32_t time, bool active_high);
void IRsendRMT_flush(uint16_t pin);

#else

static inline void IRsendRMT_begin(uint16_t) {}
static inline void IRsendRMT_enableIROut(uint16_t, uint32_t, uint8_t, bool,
                                         bool) {}
static inline uint16_t IRsendRMT_mark(uint16_t, uint16_t, bool, bool,
                                      uint32_t)
{
  return 1;
}
static inline void IRsendRMT_space(uint16_t, uint32_t, bool) {}
static inline void IRsendRMT_flush(uint16_t) {}

#endif // ESP32

#endif // IRSENDRMT_H_
