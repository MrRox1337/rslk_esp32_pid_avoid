#include "encoders.h"
#include "config.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

static constexpr int PCNT_LIMIT = 30000;
static pcnt_unit_handle_t pcntL = nullptr, pcntR = nullptr;

static pcnt_unit_handle_t setupEncoder(int pinA, int pinB) {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -PCNT_LIMIT;
  ucfg.high_limit =  PCNT_LIMIT;
  ucfg.flags.accum_count = 1;                 // keep counting past the 16-bit limit
  pcnt_unit_handle_t unit;
  ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &unit));

  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = 1000;                  // ignore pulses shorter than 1 us
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &fcfg));

  pcnt_chan_config_t ca = {};
  ca.edge_gpio_num = pinA; ca.level_gpio_num = pinB;
  pcnt_channel_handle_t chA;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &ca, &chA));

  pcnt_chan_config_t cb = {};
  cb.edge_gpio_num = pinB; cb.level_gpio_num = pinA;
  pcnt_channel_handle_t chB;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &cb, &chB));

  // Full quadrature decoding (both edges of both channels)
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // Watch points at the limits are required for accum_count to work
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit,  PCNT_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -PCNT_LIMIT));

  gpio_pullup_en((gpio_num_t)pinA);           // weak pull-ups, harmless if encoder drives the line
  gpio_pullup_en((gpio_num_t)pinB);

  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  return unit;
}

void encodersBegin() {
  pcntL = setupEncoder(ELA, ELB);
  pcntR = setupEncoder(ERA, ERB);
}

void readEncoders(long &l, long &r) {
  int cl = 0, cr = 0;
  pcnt_unit_get_count(pcntL, &cl);
  pcnt_unit_get_count(pcntR, &cr);
  l = cl; r = cr;
}
