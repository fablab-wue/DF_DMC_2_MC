#pragma once

// Live DMX512 on GP0. Hardware Serial2 cannot use GP0 (UART1 pins are 4/8/20/24),
// so this is a PIO UART at 250000 8N2 plus BREAK/MAB. Needs MAX485 for a real universe.

#include "config.h"

#include <SerialPIO.h>
#include <cstdint>

namespace sliderdmc {

class DmxEngine {
 public:
  void begin();
  bool ramping() const { return ramping_; }
  void apply(uint16_t startChannel, const uint8_t* levels, uint16_t count, bool ramp);
  void update();

 private:
  void sendNow();

  SerialPIO uart_{kDmxTxPin, NOPIN};
  uint8_t current_[kDmxChannels]{};
  uint8_t target_[kDmxChannels]{};
  uint8_t start_[kDmxChannels]{};
  bool ramping_ = false;
  bool txReady_ = false;
  uint32_t rampStartMs_ = 0;
  uint32_t lastSendMs_ = 0;
};

}  // namespace sliderdmc
