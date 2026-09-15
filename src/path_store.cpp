#include "path_store.h"

namespace sliderdmc {

void PathStore::beginUpload(int32_t startFrame, int32_t endFrame, int axisCount) {
  startFrame_ = startFrame < 1 ? 1 : startFrame;
  endFrame_ = endFrame < startFrame_ ? startFrame_ : endFrame;
  frameCount_ = endFrame_ - startFrame_ + 1;
  if (frameCount_ > kMaxUploadFrames) {
    frameCount_ = kMaxUploadFrames;
    endFrame_ = startFrame_ + frameCount_ - 1;
  }
  axisCount_ = axisCount < 1 ? 1 : (axisCount > kMaxAxes ? kMaxAxes : axisCount);
  sampleCount_ = 0;
  memset(pos_, 0, sizeof(pos_));
  memset(pd_, 0, sizeof(pd_));
  memset(frameToSample_, 0, sizeof(frameToSample_));
  memset(triggers_, 0, sizeof(triggers_));
  triggerMask_ = 0;
  built_ = false;
}

bool PathStore::storeAxis(int motor1, uint32_t index, const int32_t* values, int n, bool finalFill) {
  if (motor1 < 1 || motor1 > axisCount_ || frameCount_ <= 0) {
    return false;
  }
  int idx = static_cast<int>(index);
  if (idx < 0 || idx >= frameCount_) {
    return false;
  }
  const int axis = motor1 - 1;
  for (int i = 0; i < n && idx < frameCount_; ++i, ++idx) {
    pos_[axis][idx] = values[i];
  }
  if (finalFill && idx > 0) {
    const int32_t last = pos_[axis][idx - 1];
    while (idx < frameCount_) {
      pos_[axis][idx++] = last;
    }
  }
  return true;
}

void PathStore::storeTrigger(uint32_t mask, uint32_t index, uint32_t value) {
  triggerMask_ = mask;
  if (index < static_cast<uint32_t>(frameCount_)) {
    triggers_[index] = static_cast<uint8_t>(value & 0x0FU);
  }
}

bool PathStore::buildPd() {
  sampleCount_ = 0;
  if (frameCount_ <= 0) {
    return false;
  }
  for (int f = 0; f < frameCount_; ++f) {
    frameToSample_[f] = static_cast<uint16_t>(sampleCount_);
    int32_t delta[kMaxAxes] = {};
    int32_t maxAbs = 0;
    if (f + 1 < frameCount_) {
      for (int a = 0; a < axisCount_; ++a) {
        delta[a] = pos_[a][f + 1] - pos_[a][f];
        const int32_t ad = delta[a] < 0 ? -delta[a] : delta[a];
        if (ad > maxAbs) {
          maxAbs = ad;
        }
      }
    }
    int splits = 1;
    if (maxAbs > 32767) {
      splits = static_cast<int>((maxAbs + 32766) / 32767);
    }
    if (sampleCount_ + splits > kMaxPathSamples) {
      return false;
    }
    for (int s = 0; s < splits; ++s) {
      for (int a = 0; a < axisCount_; ++a) {
        int32_t part = delta[a] / splits;
        if (s == splits - 1) {
          part = delta[a] - part * (splits - 1);
        }
        if (part > 32767) {
          part = 32767;
        }
        if (part < -32768) {
          part = -32768;
        }
        pd_[a][sampleCount_] = static_cast<int16_t>(part);
      }
      ++sampleCount_;
    }
  }
  built_ = sampleCount_ > 0;
  return built_;
}

int32_t PathStore::positionSteps(int axis0, int localFrame) const {
  if (axis0 < 0 || axis0 >= axisCount_ || localFrame < 0 || localFrame >= frameCount_) {
    return 0;
  }
  return pos_[axis0][localFrame];
}

bool PathStore::localFrame(int dfFrame, int* out) const {
  if (out == nullptr || dfFrame < startFrame_ || dfFrame > endFrame_) {
    return false;
  }
  *out = dfFrame - startFrame_;
  return true;
}

bool PathStore::sampleRangeForFrames(int dfStart, int dfEnd, int* mcStart0, int* mcEnd0) const {
  int a = 0;
  int b = 0;
  if (mcStart0 == nullptr || mcEnd0 == nullptr || !localFrame(dfStart, &a) || !localFrame(dfEnd, &b)) {
    return false;
  }
  auto firstOf = [this](int f) { return static_cast<int>(frameToSample_[f]); };
  auto lastOf = [this](int f) {
    if (f + 1 < frameCount_) {
      return static_cast<int>(frameToSample_[f + 1]) - 1;
    }
    return sampleCount_ - 1;
  };
  int s0 = 0;
  int s1 = 0;
  if (a <= b) {
    s0 = firstOf(a);
    s1 = lastOf(b);
    if (s1 < s0) {
      s1 = s0;
    }
  } else {
    s0 = lastOf(a);
    s1 = firstOf(b);
    if (s0 < s1) {
      s0 = s1;
    }
  }
  *mcStart0 = s0;
  *mcEnd0 = s1;
  return true;
}

void PathStore::fillPd(int sample, int16_t* out) const {
  for (int a = 0; a < axisCount_; ++a) {
    out[a] = pd_[a][sample];
  }
}

uint8_t PathStore::triggerAtLocal(int localFrame) const {
  if (localFrame < 0 || localFrame >= frameCount_) {
    return 0;
  }
  return triggers_[localFrame];
}

}  // namespace sliderdmc
