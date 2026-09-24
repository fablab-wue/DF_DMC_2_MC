#include "path_store.h"

#include <dmc_protocol.h>

#include <cstring>

namespace sliderdmc {

static_assert(kMaxAxes == dfdmc::kMaxAxes, "DFDMC_MAX_AXES must match kMaxAxes");
static_assert(kMaxUploadFrames == dfdmc::kMaxUploadFrames, "upload frame cap must match the common path table");
static_assert(kDmxChannels == dfdmc::kDmxChannels, "DMX channel count must match the common engine");

void PathStore::beginUpload(int32_t startFrame, int32_t endFrame, int axisCount) {
  if (axisCount < 1) {
    axisCount = 1;
  }
  if (axisCount > kMaxAxes) {
    axisCount = kMaxAxes;
  }
  table_.beginUpload(startFrame, endFrame, axisCount);
  sampleCount_ = 0;
  memset(pd_, 0, sizeof(pd_));
  memset(frameToSample_, 0, sizeof(frameToSample_));
  built_ = false;
}

bool PathStore::storeAxis(int motor1, uint32_t index, const int32_t* values, int n, bool finalFill) {
  return table_.storeAxis(motor1, index, values, n, finalFill);
}

void PathStore::storeTrigger(uint32_t mask, uint32_t index, uint32_t value) {
  table_.storeTrigger(mask, index, value);
}

bool PathStore::buildPd() {
  sampleCount_ = 0;
  const int frameCount = table_.frameCount();
  const int axisCount = table_.axisCount();
  if (frameCount <= 0) {
    return false;
  }
  for (int f = 0; f < frameCount; ++f) {
    frameToSample_[f] = static_cast<uint16_t>(sampleCount_);
    int32_t delta[kMaxAxes] = {};
    int32_t maxAbs = 0;
    if (f + 1 < frameCount) {
      for (int a = 0; a < axisCount; ++a) {
        delta[a] = table_.positionSteps(a, f + 1) - table_.positionSteps(a, f);
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
      for (int a = 0; a < axisCount; ++a) {
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
  return table_.positionSteps(axis0, localFrame);
}

bool PathStore::localFrame(int dfFrame, int* out) const { return table_.localFrame(dfFrame, out); }

bool PathStore::sampleRangeForFrames(int dfStart, int dfEnd, int* mcStart0, int* mcEnd0) const {
  int a = 0;
  int b = 0;
  if (mcStart0 == nullptr || mcEnd0 == nullptr || !localFrame(dfStart, &a) || !localFrame(dfEnd, &b)) {
    return false;
  }
  const int frameCount = table_.frameCount();
  auto firstOf = [this](int f) { return static_cast<int>(frameToSample_[f]); };
  auto lastOf = [this, frameCount](int f) {
    if (f + 1 < frameCount) {
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
  for (int a = 0; a < table_.axisCount(); ++a) {
    out[a] = pd_[a][sample];
  }
}

uint8_t PathStore::triggerAtLocal(int localFrame) const { return table_.triggerAtLocal(localFrame); }

}  // namespace sliderdmc
