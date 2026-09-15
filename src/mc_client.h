#pragma once

// SliderMC ASCII UART on GP12/13: VH, CG, IP, MT, PD/PS/PG, simulator fallback.

#include "config.h"
#include "dmc_protocol.h"
#include "path_store.h"

#include <Arduino.h>

namespace sliderdmc {

struct McConfig {
  int axisCount = 1;
  int motorCount = 1;
  int servoCount = 0;
  float maxSpeed[kMaxAxes] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
  float maxAccel[kMaxAxes] = {50.0f, 50.0f, 50.0f, 50.0f, 50.0f, 50.0f};
  float initSpeed = 40.0f;
  float initAccel = 20.0f;
  float axisMin[kMaxAxes] = {0, 0, 0, 0, 0, 0};
  float axisMax[kMaxAxes] = {1000, 1000, 1000, 1000, 1000, 1000};
  float stepsPerUnit[kMaxAxes] = {80, 80, 80, 80, 80, 80};
  char unitName[16] = "mm";
};

class McSerialClient {
 public:
  McSerialClient() = default;

  void begin();
  void attachPath(PathStore* path);
  void sendLine(const String& line);
  void sendCommand(const String& command, const String& argument = String());
  void requestConfig();
  int advertisedMotors() const;

  void moveAxisToSteps(int axis0, int32_t steps);
  void stopMotion();
  void resetAxisSteps(int axis0, int32_t steps);
  void setSpeedSteps(int axis0, int32_t velSteps, int32_t accelSteps);
  void setLimitsSteps(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper);
  void configure(int axis0, uint8_t flags);
  void cameraTrigger(unsigned ms);
  void beep(unsigned ms);
  void requestPosition();
  void startPathStream();
  bool pathStreamDone() const { return !pdStreaming_; }
  void setPathSliceUs(uint32_t us);
  void moveToFramePose(int dfFrame);
  void pathGoRange(int dfStart, int dfEnd);

  const McConfig& config() const { return config_; }
  bool mcPresent() const { return mcPresent_; }
  bool simulatorActive() const { return simulated_; }
  bool ready() const { return mcPresent_ || simulated_; }
  bool startupTimedOut() const { return startupChecked_ && !mcPresent_ && !simulated_; }
  uint32_t movingMask() const { return movingMask_; }
  bool moving() const { return movingMask_ != 0; }
  bool pathActive() const { return pathActive_; }
  bool hardStopLatched();
  int currentFrame() const { return currentFrame_; }
  void setCurrentFrame(int f) { currentFrame_ = f; }
  int32_t positionSteps(int axis0) const;
  void update();

 private:
  void parsePackedFloats(const String& body, float* out, int n);
  void markPresent();
  void applyLiveSettings();
  void startSimulation();
  void simulateCgDump();
  void simulateCommand(const String& line);
  void pumpPathStream();
  void updateSimMotion();
  void updateSimPath();
  void handleMcLine(const String& line);
  void parseConfigLine(const String& line);

  String rxBuffer_;
  McConfig config_{};
  PathStore* path_ = nullptr;
  bool mcPresent_ = false;
  bool startupChecked_ = false;
  bool simulated_ = false;
  bool liveApplied_ = false;
  bool configDrain_ = false;
  bool hardStop_ = false;
  bool pathActive_ = false;
  bool pdStreaming_ = false;
  uint32_t movingMask_ = 0;
  uint32_t startupDeadlineMs_ = 0;
  uint32_t lastMotionMs_ = 0;
  uint32_t lastCgMs_ = 0;
  uint32_t pathSliceUs_ = 41667;
  int pdStreamIndex_ = 0;
  int currentFrame_ = 1;
  int playStartFrame_ = 1;
  int playEndFrame_ = 1;
  int simPgA_ = 1;
  int simPgB_ = 1;
  int simPathSample_ = 0;
  uint32_t simPathNextMs_ = 0;
  float sessionSpeed_ = 40.0f;
  float sessionAccel_ = 20.0f;
  float positionMm_[kMaxAxes]{};
  float targetMm_[kMaxAxes]{};
  uint8_t motorConfig_[kMaxAxes]{};
};

}  // namespace sliderdmc
