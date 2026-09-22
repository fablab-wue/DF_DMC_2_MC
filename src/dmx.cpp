#include "dmx.h"

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#include <cstring>

namespace sliderdmc {

void DmxEngine::begin() {
  memset(current_, 0, sizeof(current_));
  memset(target_, 0, sizeof(target_));
  memset(start_, 0, sizeof(start_));
  ramping_ = false;
  lastSendMs_ = 0;
  uart_.begin(250000, SERIAL_8N2);
  txReady_ = static_cast<bool>(uart_);
  beginPwm();
  writePwm();
}

void DmxEngine::apply(uint16_t startChannel, const uint8_t* levels, uint16_t count, bool ramp) {
  if (startChannel < 1) {
    startChannel = 1;
  }
  const int start = static_cast<int>(startChannel - 1);
  if (start >= kDmxChannels || levels == nullptr || count == 0) {
    return;
  }
  if (start + count > kDmxChannels) {
    count = static_cast<uint16_t>(kDmxChannels - start);
  }
  memcpy(start_, current_, sizeof(current_));
  memcpy(target_, current_, sizeof(target_));
  memcpy(target_ + start, levels, count);
  if (!ramp) {
    memcpy(current_, target_, sizeof(current_));
    ramping_ = false;
    sendNow();
    return;
  }
  rampStartMs_ = millis();
  ramping_ = true;
}

void DmxEngine::update() {
  const uint32_t now = millis();
  if (!ramping_) {
    return;
  }
  const uint32_t elapsed = now - rampStartMs_;
  constexpr uint32_t kRampMs = 500;
  float t = static_cast<float>(elapsed) / static_cast<float>(kRampMs);
  if (t >= 1.0f) {
    memcpy(current_, target_, sizeof(current_));
    ramping_ = false;
    sendNow();
    return;
  }
  for (int i = 0; i < kDmxChannels; ++i) {
    const int a = start_[i];
    const int b = target_[i];
    current_[i] = static_cast<uint8_t>(a + static_cast<int>((b - a) * t));
  }
  if (now - lastSendMs_ >= 25) {
    sendNow();
  }
}

void DmxEngine::beginPwm() {
  const float div =
      static_cast<float>(clock_get_hz(clk_sys)) / (static_cast<float>(kDmxPwmHz) * 255.0f);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, div);
  pwm_config_set_wrap(&cfg, 254);
  bool sliceInited[8] = {};
  for (uint8_t pin : kDmxPwmPins) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    const unsigned slice = pwm_gpio_to_slice_num(pin);
    if (slice < 8 && !sliceInited[slice]) {
      pwm_init(slice, &cfg, true);
      sliceInited[slice] = true;
    }
    pwm_set_gpio_level(pin, 0);
  }
}

void DmxEngine::writePwm() {
  for (int i = 0; i < 6; ++i) {
    pwm_set_gpio_level(kDmxPwmPins[i], current_[i]);
  }
}

void DmxEngine::sendNow() {
  writePwm();
  if (!txReady_) {
    return;
  }
  uart_.flush();
  uart_.end();
  pinMode(kDmxTxPin, OUTPUT);
  digitalWrite(kDmxTxPin, LOW);
  delayMicroseconds(88);
  digitalWrite(kDmxTxPin, HIGH);
  delayMicroseconds(12);
  uart_.begin(250000, SERIAL_8N2);
  txReady_ = static_cast<bool>(uart_);
  if (!txReady_) {
    return;
  }
  uart_.write(static_cast<uint8_t>(0));
  uart_.write(current_, kDmxChannels);
  lastSendMs_ = millis();
}

}  // namespace sliderdmc
