#pragma once

// Onboard status: classic GPIO LED or WS2812 on GP16 (SLIDERDMC_STATUS_LED_MODE).

#include "config.h"

#include <Adafruit_NeoPixel.h>
#include <cstdint>

namespace sliderdmc {

class StatusLed {
 public:
  enum class State {
    kBoot,
    kWaitingForDf,
    kDfConnected,
    kWaitingForMc,
    kMcReady,
    kDmcActivity,
    kError,
  };

  void begin();
  void markActivity();
  void update(bool dfConnected, bool mcPresent, bool simulator, bool startupTimedOut);

 private:
  void setColor(uint8_t r, uint8_t g, uint8_t b);
  void testSequence();

  Adafruit_NeoPixel pixels_{1, kNeoPixelPin, NEO_GRB + NEO_KHZ800};
  uint8_t lastR_ = 255;
  uint8_t lastG_ = 255;
  uint8_t lastB_ = 255;
  uint32_t lastHeartbeatMs_ = 0;
  uint32_t stateStartedMs_ = 0;
  uint32_t lastActivityMs_ = 0;
  bool heartbeatState_ = false;
  State state_ = State::kBoot;
};

}  // namespace sliderdmc
