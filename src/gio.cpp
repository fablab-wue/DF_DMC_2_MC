#include "gio.h"

namespace sliderdmc {

static void ocWrite(uint8_t pin, bool active) {
  if (active) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void DmcGio::begin() {
  for (uint8_t pin : kGioOutPins) {
    ocWrite(pin, false);
  }
  for (uint8_t pin : kGioInPins) {
    pinMode(pin, INPUT_PULLUP);
  }
  ocWrite(kCameraShutterPin, false);
  pinMode(kBuzzerPin, OUTPUT);
  digitalWrite(kBuzzerPin, LOW);
  outBits_ = 0;
  inBits_ = readInputs();
  inStable_ = inBits_;
  debounce_ = 0;
  camShutter_ = false;
  buzzerRemainMs_ = 0;
  lastTickMs_ = millis();
}

void DmcGio::setOutputs(uint32_t bits) {
  outBits_ = bits & 0x0FU;
  for (int i = 0; i < 4; ++i) {
    ocWrite(kGioOutPins[i], (outBits_ & (1U << i)) != 0);
  }
}

bool DmcGio::pollInputChange() {
  const uint32_t now = readInputs();
  if (now == inBits_) {
    if (debounce_ < 5) {
      ++debounce_;
    }
    if (debounce_ >= 5 && now != inStable_) {
      inStable_ = now;
      return true;
    }
  } else {
    inBits_ = now;
    debounce_ = 0;
  }
  return false;
}

void DmcGio::setCameraShutter(bool on) {
  camShutter_ = on;
  ocWrite(kCameraShutterPin, on);
}

void DmcGio::pulseBuzzer(unsigned ms) {
  if (ms == 0) {
    return;
  }
  digitalWrite(kBuzzerPin, HIGH);
  buzzerRemainMs_ = ms;
}

void DmcGio::tick() {
  const uint32_t now = millis();
  const uint32_t dt = now - lastTickMs_;
  lastTickMs_ = now;
  if (buzzerRemainMs_ == 0) {
    return;
  }
  if (dt >= buzzerRemainMs_) {
    buzzerRemainMs_ = 0;
    digitalWrite(kBuzzerPin, LOW);
  } else {
    buzzerRemainMs_ -= dt;
  }
}

uint32_t DmcGio::readInputs() const {
  uint32_t bits = 0;
  for (int i = 0; i < 4; ++i) {
    if (digitalRead(kGioInPins[i]) == LOW) {
      bits |= (1U << i);
    }
  }
  return bits;
}

}  // namespace sliderdmc
