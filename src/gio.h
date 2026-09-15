#pragma once

// DragonFrame GIO on this USB Pico (not SliderMC extender pins).

#include "config.h"

#include <cstdint>

namespace sliderdmc {

class DmcGio {
 public:
  void begin();
  void setOutputs(uint32_t bits);
  uint32_t outputs() const { return outBits_; }
  uint32_t inputs() const { return inStable_; }
  bool pollInputChange();
  void setCameraShutter(bool on);
  bool cameraShutter() const { return camShutter_; }
  void pulseBuzzer(unsigned ms);
  void tick();

 private:
  uint32_t readInputs() const;

  uint32_t outBits_ = 0;
  uint32_t inBits_ = 0;
  uint32_t inStable_ = 0;
  int debounce_ = 0;
  bool camShutter_ = false;
  unsigned buzzerRemainMs_ = 0;
  uint32_t lastTickMs_ = 0;
};

}  // namespace sliderdmc
