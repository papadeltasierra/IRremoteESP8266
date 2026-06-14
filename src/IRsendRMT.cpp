// Copyright 2026 GitHub Copilot

#ifdef ESP32

#include "IRsendRMT.h"

#include <Arduino.h>
#include <driver/rmt.h>

namespace {
static constexpr uint16_t kMaxRmtItems = 512;

struct RMTState {
  bool active = false;
  bool installed = false;
  bool carrier_enabled = false;
  uint16_t pin = 0;
  rmt_channel_t channel = RMT_CHANNEL_MAX;
  bool active_high = true;
  bool modulation = false;
  uint32_t period = 0;
  uint16_t item_count = 0;
  rmt_item32_t items[kMaxRmtItems];
};

static RMTState g_rmt_state[RMT_CHANNEL_MAX];

static RMTState* FindState(uint16_t pin) {
  for (auto& state : g_rmt_state) {
    if (state.active && state.pin == pin) return &state;
  }
  return nullptr;
}

static RMTState* AllocateState(uint16_t pin) {
  for (auto& state : g_rmt_state) {
    if (!state.active) {
      state.active = true;
      state.pin = pin;
      state.channel = static_cast<rmt_channel_t>(&state - g_rmt_state);
      return &state;
    }
  }
  return nullptr;
}

static void ResetRmtBuffer(RMTState* state) { state->item_count = 0; }

static void FlushRmtBuffer(RMTState* state) {
  if (state->item_count == 0) return;
  rmt_write_items(state->channel, state->items, state->item_count, true);
  rmt_wait_tx_done(state->channel, portMAX_DELAY);
  state->item_count = 0;
}

static void AppendRmtItem(RMTState* state, const rmt_item32_t& item) {
  if (state->item_count >= kMaxRmtItems) {
    FlushRmtBuffer(state);
  }
  state->items[state->item_count++] = item;
}

static void AppendRmtDuration(RMTState* state, uint32_t duration,
                              uint8_t level) {
  while (duration > 0) {
    uint16_t chunk =
        duration > 32767U ? 32767U : static_cast<uint16_t>(duration);
    rmt_item32_t item;
    item.duration0 = chunk;
    item.level0 = level;
    item.duration1 = 0;
    item.level1 = 0;
    AppendRmtItem(state, item);
    duration -= chunk;
  }
}

static void ConfigureRmtChannel(RMTState* state, uint16_t pin, uint32_t freq,
                                uint8_t duty, bool active_high) {
  rmt_config_t config =
      RMT_DEFAULT_CONFIG_TX(static_cast<gpio_num_t>(pin), state->channel);
  config.clk_div = 80;  // 1 MHz tick resolution.
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = state->carrier_enabled;
  config.tx_config.carrier_freq_hz = freq;
  config.tx_config.carrier_duty_percent = duty;
  config.tx_config.carrier_level =
      active_high ? RMT_CARRIER_LEVEL_HIGH : RMT_CARRIER_LEVEL_LOW;
  config.tx_config.idle_level =
      active_high ? RMT_IDLE_LEVEL_LOW : RMT_IDLE_LEVEL_HIGH;
  config.tx_config.idle_output_en = true;

  rmt_config(&config);

  if (!state->installed) {
    rmt_driver_install(state->channel, 0, 0);
    state->installed = true;
  }

  rmt_set_gpio(state->channel, RMT_MODE_TX, static_cast<gpio_num_t>(pin),
               false);
}

}  // namespace

void IRsendRMT_begin(uint16_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void IRsendRMT_enableIROut(uint16_t pin, uint32_t freq, uint8_t duty,
                           bool modulation, bool active_high) {
  RMTState* state = FindState(pin);
  if (state == nullptr) state = AllocateState(pin);
  if (state == nullptr) return;

  if (freq < 1000) freq *= 1000;
  state->modulation = modulation;
  state->active_high = active_high;
  state->carrier_enabled = modulation && duty < 100;
  state->period = (1000000UL + freq / 2) / freq;
  ConfigureRmtChannel(state, pin, freq, duty, active_high);
  ResetRmtBuffer(state);
}

uint16_t IRsendRMT_mark(uint16_t pin, uint16_t usec, bool modulation,
                        bool active_high, uint32_t period) {
  RMTState* state = FindState(pin);
  if (state == nullptr || usec == 0) {
    digitalWrite(pin, active_high ? HIGH : LOW);
    delayMicroseconds(usec);
    digitalWrite(pin, active_high ? LOW : HIGH);
    return period ? static_cast<uint16_t>((usec + period - 1) / period) : 1;
  }

  AppendRmtDuration(state, usec, active_high ? 1 : 0);
  return period ? static_cast<uint16_t>((usec + period - 1) / period) : 1;
}

void IRsendRMT_space(uint16_t pin, uint32_t time, bool active_high) {
  RMTState* state = FindState(pin);
  if (state == nullptr || time == 0) {
    if (state == nullptr) {
      digitalWrite(pin, active_high ? LOW : HIGH);
      if (time == 0) return;
      delayMicroseconds(time);
    }
    return;
  }

  AppendRmtDuration(state, time, active_high ? 0 : 1);
}

void IRsendRMT_flush(uint16_t pin) {
  RMTState* state = FindState(pin);
  if (state == nullptr) return;
  FlushRmtBuffer(state);
}

#endif  // ESP32
