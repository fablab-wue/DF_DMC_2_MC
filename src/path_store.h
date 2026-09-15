#pragma once

// DragonFrame upload table → SliderMC PD samples (µm int16, split if needed).
// PG ranges are 0-based inclusive; start>end means reverse.

#include "config.h"

#include <cstdint>
#include <cstring>

namespace sliderdmc {

class PathStore {
 public:
  void beginUpload(int32_t startFrame, int32_t endFrame, int axisCount);
  bool storeAxis(int motor1, uint32_t index, const int32_t* values, int n, bool finalFill);
  void storeTrigger(uint32_t mask, uint32_t index, uint32_t value);
  bool buildPd();

  bool empty() const { return !built_ || sampleCount_ <= 0; }
  int frameCount() const { return frameCount_; }
  int sampleCount() const { return sampleCount_; }
  int startFrame() const { return startFrame_; }
  int endFrame() const { return endFrame_; }
  int axisCount() const { return axisCount_; }
  uint32_t triggerMask() const { return triggerMask_; }

  int32_t positionSteps(int axis0, int localFrame) const;
  bool localFrame(int dfFrame, int* out) const;
  bool sampleRangeForFrames(int dfStart, int dfEnd, int* mcStart0, int* mcEnd0) const;
  void fillPd(int sample, int16_t* out) const;
  uint8_t triggerAtLocal(int localFrame) const;

 private:
  int32_t pos_[kMaxAxes][kMaxUploadFrames]{};
  int16_t pd_[kMaxAxes][kMaxPathSamples]{};
  uint16_t frameToSample_[kMaxUploadFrames]{};
  uint8_t triggers_[kMaxUploadFrames]{};
  int frameCount_ = 0;
  int sampleCount_ = 0;
  int startFrame_ = 1;
  int endFrame_ = 1;
  int axisCount_ = 1;
  uint32_t triggerMask_ = 0;
  bool built_ = false;
};

}  // namespace sliderdmc
