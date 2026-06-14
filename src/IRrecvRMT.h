// Copyright 2026 GitHub Copilot

#ifndef IRRECVRMT_H_
#define IRRECVRMT_H_

#include <stdint.h>

#ifdef ESP32

void IRrecvRMT_begin(uint16_t pin);
void IRrecvRMT_enableIRIn(uint16_t pin, uint16_t bufsize, uint16_t timeout_ms);
void IRrecvRMT_disableIRIn(uint16_t pin);
void IRrecvRMT_pause(uint16_t pin);
void IRrecvRMT_resume(uint16_t pin);
void IRrecvRMT_poll(uint16_t pin, uint16_t *rawbuf, uint16_t bufsize,
                    volatile uint16_t &rawlen, volatile uint8_t &overflow,
                    volatile uint8_t &rcvstate, const uint8_t timeout_ms);

#else

static inline void IRrecvRMT_begin(uint16_t) {}
static inline void IRrecvRMT_enableIRIn(uint16_t, uint16_t, uint16_t) {}
static inline void IRrecvRMT_disableIRIn(uint16_t) {}
static inline void IRrecvRMT_pause(uint16_t) {}
static inline void IRrecvRMT_resume(uint16_t) {}
static inline void IRrecvRMT_poll(uint16_t, uint16_t *, uint16_t,
                                  volatile uint16_t &, volatile uint8_t &,
                                  volatile uint8_t &, const uint8_t) {}

#endif // ESP32

#endif // IRRECVRMT_H_
