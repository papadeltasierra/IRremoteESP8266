// Copyright 2026 GitHub Copilot

#ifdef ESP32

#include "IRrecvRMT.h"

#include <Arduino.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

#include <algorithm>

#include "IRrecv.h"

namespace
{

  constexpr uint16_t kRawTick = 2; // Raw buffer units are 2us ticks.
  constexpr size_t kRxRingBufferSize = 1024;

  struct RMTRecvState
  {
    bool active = false;
    bool installed = false;
    bool capturing = false;
    bool use_gpio_fallback = false;
    uint16_t pin = 0;
    uint16_t bufsize = 0;
    uint32_t timeout_us = 0;
    rmt_channel_t channel = RMT_CHANNEL_MAX;
    RingbufHandle_t ringbuf = nullptr;
    uint32_t last_transition_us = 0;
  };

  static RMTRecvState g_rmt_recv_state[RMT_CHANNEL_MAX];
  static portMUX_TYPE rmt_recv_mux = portMUX_INITIALIZER_UNLOCKED;

  static RMTRecvState *FindState(uint16_t pin)
  {
    for (auto &state : g_rmt_recv_state)
    {
      if (state.active && state.pin == pin)
        return &state;
    }
    return nullptr;
  }

  static RMTRecvState *AllocateState(uint16_t pin)
  {
    for (uint32_t i = 0; i < RMT_CHANNEL_MAX; ++i)
    {
      auto &state = g_rmt_recv_state[i];
      if (!state.active)
      {
        state.active = true;
        state.pin = pin;
        state.channel = static_cast<rmt_channel_t>(i);
        return &state;
      }
    }
    return nullptr;
  }

  static uint16_t RawTicksFromUsec(uint32_t usec)
  {
    // Round up to preserve short pulses.
    return static_cast<uint16_t>(std::min<uint32_t>(
        (usec + kRawTick - 1) / kRawTick, static_cast<uint32_t>(UINT16_MAX)));
  }

  static void ResetRingBuffer(RMTRecvState *state)
  {
    if (state == nullptr || state->ringbuf == nullptr)
      return;
    size_t item_size;
    void *item = nullptr;
    while ((item = xRingbufferReceive(state->ringbuf, &item_size, 0)) !=
           nullptr)
    {
      vRingbufferReturnItem(state->ringbuf, item);
    }
  }

  static void StopRmtReception(RMTRecvState *state)
  {
    if (state == nullptr || !state->capturing)
      return;
    rmt_rx_stop(state->channel);
    state->capturing = false;
  }

  static void StartRmtReception(RMTRecvState *state)
  {
    if (state == nullptr || state->ringbuf == nullptr || state->capturing)
      return;
    ResetRingBuffer(state);
    rmt_rx_start(state->channel, true);
    state->capturing = true;
  }

  static bool IsMarkLevel(uint32_t level)
  {
    // Most IR receivers output LOW while sensing an IR pulse.
    return level == 0;
  }

} // namespace

namespace _IRrecv
{
  extern atomic_irparams_t params;
}

namespace
{

  // RMT idle detection ISR - triggers when rx signal goes idle.
  static void IRAM_ATTR RmtIsrHandler(void *arg)
  {
    RMTRecvState *state = static_cast<RMTRecvState *>(arg);
    if (state != nullptr && state->capturing)
    {
      portENTER_CRITICAL_ISR(&rmt_recv_mux);
      if (_IRrecv::params.rawlen > 0)
      {
        _IRrecv::params.rcvstate = kStopState;
      }
      portEXIT_CRITICAL_ISR(&rmt_recv_mux);
      rmt_rx_stop(state->channel);
      state->capturing = false;
    }
  }

  // GPIO interrupt fallback for faster idle detection.
  static void IRAM_ATTR GpioIsrFallback(void *arg)
  {
    RMTRecvState *state = static_cast<RMTRecvState *>(arg);
    if (state == nullptr)
      return;
    state->last_transition_us = micros();
  }

  static void ParseRmtItemsIntoRawbuf(RMTRecvState *state, uint16_t *rawbuf,
                                      uint16_t bufsize, volatile uint16_t &rawlen,
                                      volatile uint8_t &overflow)
  {
    if (state == nullptr || state->ringbuf == nullptr || rawbuf == nullptr)
      return;

    size_t item_size;
    auto *items = static_cast<rmt_item32_t *>(
        xRingbufferReceive(state->ringbuf, &item_size, 0));
    if (items == nullptr)
      return;

    const size_t item_count = item_size / sizeof(rmt_item32_t);
    bool have_mark = false;
    bool last_level_mark = false;

    for (size_t i = 0; i < item_count; ++i)
    {
      const auto &item = items[i];
      const uint32_t durations[2] = {item.duration0, item.duration1};
      const uint32_t levels[2] = {item.level0, item.level1};

      for (size_t part = 0; part < 2; ++part)
      {
        uint32_t duration = durations[part];
        if (duration == 0)
          continue;

        bool is_mark = IsMarkLevel(levels[part]);
        if (!have_mark)
        {
          if (!is_mark)
            continue; // Skip leading idle space.
          have_mark = true;
          rawlen = 1;
          rawbuf[0] =
              1; // Dummy entry to match the existing decoder start offset.
          last_level_mark = true;
        }

        uint16_t ticks = RawTicksFromUsec(duration);
        if (rawlen == 1)
        {
          if (rawlen + 1 >= bufsize)
          {
            overflow = true;
            break;
          }
          rawbuf[rawlen++] = ticks;
          last_level_mark = is_mark;
          continue;
        }

        if (is_mark == last_level_mark)
        {
          uint32_t extended = static_cast<uint32_t>(rawbuf[rawlen - 1]) + ticks;
          rawbuf[rawlen - 1] =
              static_cast<uint16_t>(std::min<uint32_t>(extended, UINT16_MAX));
        }
        else
        {
          if (rawlen >= bufsize)
          {
            overflow = true;
            break;
          }
          rawbuf[rawlen++] = ticks;
          last_level_mark = is_mark;
        }
      }

      if (overflow)
        break;
    }

    vRingbufferReturnItem(state->ringbuf, items);

    // Discard any remaining completed packets to keep the ring buffer fresh.
    ResetRingBuffer(state);
  }

} // namespace

void IRrecvRMT_begin(uint16_t pin) { pinMode(pin, INPUT); }

void IRrecvRMT_enableIRIn(uint16_t pin, uint16_t bufsize, uint16_t timeout_ms)
{
  RMTRecvState *state = FindState(pin);
  if (state == nullptr)
    state = AllocateState(pin);
  if (state == nullptr)
    return;

  state->bufsize = bufsize;
  state->timeout_us = static_cast<uint32_t>(timeout_ms) * 1000UL;

  rmt_config_t config =
      RMT_DEFAULT_CONFIG_RX(static_cast<gpio_num_t>(pin), state->channel);
  config.clk_div = 80; // 1 MHz tick resolution.
  config.rx_config.filter_en = false;
  config.rx_config.idle_threshold = state->timeout_us;

  rmt_config(&config);

  if (!state->installed)
  {
    rmt_driver_install(state->channel, kRxRingBufferSize, 0);
    state->installed = true;
  }

  rmt_set_gpio(state->channel, RMT_MODE_RX, static_cast<gpio_num_t>(pin),
               false);
  rmt_get_ringbuf_handle(state->channel, &state->ringbuf);

  // Register RMT ISR for idle detection (lower latency than polling).
  rmt_isr_register(RmtIsrHandler, state, 0, nullptr);

  // Optionally attach GPIO interrupt as fallback for even faster detection.
  state->use_gpio_fallback = true;
  attachInterruptArg(pin, GpioIsrFallback, state, CHANGE);

  StartRmtReception(state);
}

void IRrecvRMT_disableIRIn(uint16_t pin)
{
  RMTRecvState *state = FindState(pin);
  if (state == nullptr)
    return;
  StopRmtReception(state);
  if (state->use_gpio_fallback)
  {
    detachInterrupt(state->pin);
    state->use_gpio_fallback = false;
  }
  if (state->ringbuf != nullptr)
  {
    vRingbufferDelete(state->ringbuf);
    state->ringbuf = nullptr;
  }
  if (state->installed)
  {
    rmt_driver_uninstall(state->channel);
    state->installed = false;
  }
  state->active = false;
  state->capturing = false;
}

void IRrecvRMT_pause(uint16_t pin)
{
  RMTRecvState *state = FindState(pin);
  if (state == nullptr)
    return;
  StopRmtReception(state);
}

void IRrecvRMT_resume(uint16_t pin)
{
  RMTRecvState *state = FindState(pin);
  if (state == nullptr)
    return;
  StartRmtReception(state);
}

void IRrecvRMT_poll(uint16_t pin, uint16_t *rawbuf, uint16_t bufsize,
                    volatile uint16_t &rawlen, volatile uint8_t &overflow,
                    volatile uint8_t &rcvstate, const uint8_t timeout_ms)
{
  RMTRecvState *state = FindState(pin);
  if (state == nullptr || rawbuf == nullptr)
    return;
  if (state->ringbuf == nullptr)
    return;
  if (rcvstate == kStopState)
    return;

  ParseRmtItemsIntoRawbuf(state, rawbuf, bufsize, rawlen, overflow);

  if (rawlen > 1)
  {
    rcvstate = kStopState;
    StopRmtReception(state);
  }
}

#endif // ESP32
