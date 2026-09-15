#include "status_led.h"

namespace sliderdmc {

void StatusLed::begin() {
  if (kUseClassicLed) {
    pinMode(kStatusLedPin, OUTPUT);
  } else if (kUseNeoPixel) {
    pixels_.begin();
    pixels_.setBrightness(kNeoPixelBrightness);
    pixels_.clear();
    pixels_.show();
    lastR_ = 0;
    lastG_ = 0;
    lastB_ = 0;
    testSequence();
  }
  state_ = State::kBoot;
  stateStartedMs_ = millis();
}

void StatusLed::setColor(uint8_t r, uint8_t g, uint8_t b) {
  if (r == lastR_ && g == lastG_ && b == lastB_) {
    return;
  }
  lastR_ = r;
  lastG_ = g;
  lastB_ = b;
  pixels_.setPixelColor(0, pixels_.Color(r, g, b));
  pixels_.show();
}

void StatusLed::testSequence() {
  setColor(255, 0, 0);
  delay(300);
  setColor(0, 255, 0);
  delay(300);
  setColor(0, 0, 255);
  delay(300);
  setColor(0, 0, 0);
}

void StatusLed::markActivity() {
  lastActivityMs_ = millis();
  state_ = State::kDmcActivity;
  stateStartedMs_ = millis();
}

void StatusLed::update(bool dfConnected, bool mcPresent, bool simulator, bool startupTimedOut) {
  const uint32_t now = millis();

  if (startupTimedOut) {
    state_ = State::kError;
  } else if (state_ == State::kDmcActivity && now - lastActivityMs_ > 250) {
    if (dfConnected && (mcPresent || simulator)) {
      state_ = State::kMcReady;
    } else if (dfConnected) {
      state_ = State::kDfConnected;
    } else {
      state_ = State::kWaitingForDf;
    }
  } else if (state_ != State::kDmcActivity) {
    if (!dfConnected && now - stateStartedMs_ < 1800) {
      state_ = State::kBoot;
    } else if (!dfConnected) {
      state_ = State::kWaitingForDf;
    } else if (!mcPresent && !simulator && !startupTimedOut) {
      state_ = State::kWaitingForMc;
    } else if (mcPresent || simulator) {
      state_ = State::kMcReady;
    } else {
      state_ = State::kDfConnected;
    }
  }

  if (kUseClassicLed) {
    const uint32_t heartbeatMs = (mcPresent || simulator) ? kStatusLedHeartbeatMs : kStatusLedFastBlinkMs;
    if (now - lastHeartbeatMs_ >= heartbeatMs) {
      lastHeartbeatMs_ = now;
      heartbeatState_ = !heartbeatState_;
      digitalWrite(kStatusLedPin, heartbeatState_ ? HIGH : LOW);
    }
    return;
  }
  if (!kUseNeoPixel) {
    return;
  }
  const uint32_t pulseWindow = (state_ == State::kError)           ? 120
                               : (state_ == State::kBoot)         ? 180
                               : (state_ == State::kWaitingForDf) ? 260
                               : (state_ == State::kWaitingForMc) ? 350
                               : (state_ == State::kDmcActivity)  ? 80
                               : (state_ == State::kMcReady)      ? 500
                                                                  : 300;
  if (now - lastHeartbeatMs_ >= pulseWindow) {
    lastHeartbeatMs_ = now;
    heartbeatState_ = !heartbeatState_;
  }
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  switch (state_) {
    case State::kBoot:
      r = heartbeatState_ ? 64 : 0;
      g = heartbeatState_ ? 0 : 16;
      b = 64;
      break;
    case State::kWaitingForDf:
      r = 0;
      g = 0;
      b = heartbeatState_ ? 64 : 16;
      break;
    case State::kDfConnected:
      r = 0;
      g = heartbeatState_ ? 64 : 24;
      b = heartbeatState_ ? 32 : 16;
      break;
    case State::kWaitingForMc:
      r = heartbeatState_ ? 64 : 16;
      g = heartbeatState_ ? 24 : 8;
      b = 0;
      break;
    case State::kMcReady:
      r = 0;
      g = heartbeatState_ ? 96 : 32;
      b = 0;
      break;
    case State::kDmcActivity:
      r = 0;
      g = heartbeatState_ ? 96 : 16;
      b = heartbeatState_ ? 64 : 16;
      break;
    case State::kError:
      r = heartbeatState_ ? 120 : 16;
      g = 0;
      b = 0;
      break;
  }
  setColor(r, g, b);
}

}  // namespace sliderdmc
