#pragma once

// Dragonframe upload table → SliderMC PD samples (µm int16, split if needed).
// Positions live in dfdmc::PathTable. PG ranges are 0-based inclusive; start>end means reverse.

#include "config.h"

#include <path_table.h>

#include <cstdint>

namespace sliderdmc {

class PathStore {
 public:
  void beginUpload(int32_t startFrame, int32_t endFrame, int axisCount);
  bool storeAxis(int motor1, uint32_t index, const int32_t* values, int n, bool finalFill);
  void storeTrigger(uint32_t mask, uint32_t index, uint32_t value);
  bool buildPd();

  bool empty() const { return !built_ || sampleCount_ <= 0; }
  int frameCount() const { return table_.frameCount(); }
  int sampleCount() const { return sampleCount_; }
  int startFrame() const { return table_.startFrame(); }
  int endFrame() const { return table_.endFrame(); }
  int axisCount() const { return table_.axisCount(); }
  uint32_t triggerMask() const { return table_.triggerMask(); }

  int32_t positionSteps(int axis0, int localFrame) const;
  bool localFrame(int dfFrame, int* out) const;
  bool sampleRangeForFrames(int dfStart, int dfEnd, int* mcStart0, int* mcEnd0) const;
  void fillPd(int sample, int16_t* out) const;
  uint8_t triggerAtLocal(int localFrame) const;

 private:
  dfdmc::PathTable table_;
  int16_t pd_[kMaxAxes][kMaxPathSamples]{};
  uint16_t frameToSample_[kMaxUploadFrames]{};
  int sampleCount_ = 0;
  bool built_ = false;
};

}  // namespace sliderdmc
