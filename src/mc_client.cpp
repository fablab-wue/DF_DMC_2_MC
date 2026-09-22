#include "mc_client.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace sliderdmc {

constexpr bool kDebugEnabled = false;

template <typename T>
void debugLog(const T& value) {
  if (kDebugEnabled) {
    Serial.println(value);
  }
}

template <typename T, typename... Args>
void debugLog(const T& first, const Args&... rest) {
  if (!kDebugEnabled) {
    return;
  }
  Serial.print(first);
  (Serial.print(rest), ...);
  Serial.println();
}

String fmtMc(float v) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(v));
  return String(buf);
}

static void writeMovePin(char letter) {
  switch (letter) {
    case 'M':
    case 'A':
    case 'B':
    case 'H':
    case 'P':
      digitalWrite(kMovePin, HIGH);
      break;
    case 'I':
    case 'E':
    case 'D':
    case 'L':
      digitalWrite(kMovePin, LOW);
      break;
    default:
      break;
  }
}

String packMc(const float* v, int n) {
  String out;
  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      out += " | ";
    }
    out += fmtMc(v[i]);
  }
  return out;
}

void McSerialClient::begin() {
  Serial1.setTX(kMcUartTxPin);
  Serial1.setRX(kMcUartRxPin);
  Serial1.begin(kMcUartBaud);
  delay(50);
  sendCommand("VH");
  delay(80);
  startupDeadlineMs_ = millis() + kMcStartupTimeoutMs;
  mcPresent_ = false;
  startupChecked_ = false;
  simulated_ = false;
  liveApplied_ = false;
  configDrain_ = false;
  movingMask_ = 0;
  hardStop_ = false;
  pathActive_ = false;
  pdStreamIndex_ = 0;
  pdStreaming_ = false;
  for (int i = 0; i < kMaxAxes; ++i) {
    positionMm_[i] = 0;
    targetMm_[i] = 0;
    motorConfig_[i] = kDmcMotorConfigEnabled;
  }
  requestConfig();
}

void McSerialClient::attachPath(PathStore* path) { path_ = path; }

void McSerialClient::sendLine(const String& line) {
  if (!simulated_) {
    if (line.length() == 0) {
      Serial1.write('\n');
      return;
    }
    Serial1.write((line + "\n").c_str());
    return;
  }
  simulateCommand(line);
}

void McSerialClient::sendCommand(const String& command, const String& argument) {
  if (argument.length() == 0) {
    sendLine(command);
  } else {
    sendLine(command + " " + argument);
  }
}

void McSerialClient::requestConfig() { sendLine("CG"); }

int McSerialClient::advertisedMotors() const {
  int n = config_.axisCount;
  if (n < 1) {
    n = 1;
  }
  if (n > kMaxAxes) {
    n = kMaxAxes;
  }
  return n;
}

void McSerialClient::moveAxisToSteps(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= advertisedMotors()) {
    return;
  }
  targetMm_[axis0] = dmcStepsToMc(steps);
  movingMask_ |= (1U << axis0);
  lastMotionMs_ = millis();
  sendCommand("MT", packMc(targetMm_, advertisedMotors()));
}

void McSerialClient::stopMotion() {
  movingMask_ = 0;
  memcpy(targetMm_, positionMm_, sizeof(targetMm_));
  pathActive_ = false;
  sendCommand("MS");
}

void McSerialClient::resetAxisSteps(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= advertisedMotors()) {
    return;
  }
  positionMm_[axis0] = dmcStepsToMc(steps);
  targetMm_[axis0] = positionMm_[axis0];
  movingMask_ &= ~(1U << axis0);
  sendCommand("SP", packMc(positionMm_, advertisedMotors()));
}

void McSerialClient::setSpeedSteps(int axis0, int32_t velSteps, int32_t accelSteps) {
  float vel = dmcStepsToMc(velSteps);
  float acc = dmcStepsToMc(accelSteps);
  if (axis0 >= 0 && axis0 < kMaxAxes) {
    if (vel > config_.maxSpeed[axis0]) {
      vel = config_.maxSpeed[axis0];
    }
    if (acc > config_.maxAccel[axis0]) {
      acc = config_.maxAccel[axis0];
    }
  }
  if (vel < 0.001f) {
    vel = 0.001f;
  }
  sessionSpeed_ = vel;
  if (acc > 0) {
    sessionAccel_ = acc;
    sendCommand("SA", fmtMc(acc));
  }
  sendCommand("SS", fmtMc(vel));
}

void McSerialClient::setLimitsSteps(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper) {
  (void)axis0;
  if (lowerEn) {
    sendCommand("SL", fmtMc(dmcStepsToMc(lower)));
  }
  if (upperEn) {
    sendCommand("SR", fmtMc(dmcStepsToMc(upper)));
  }
}

void McSerialClient::configure(int axis0, uint8_t flags) {
  if (axis0 >= 0 && axis0 < kMaxAxes) {
    motorConfig_[axis0] = flags;
  }
}

void McSerialClient::cameraTrigger(unsigned ms) { sendCommand("CT", String(ms < 1 ? 100 : ms)); }

void McSerialClient::beep(unsigned ms) { sendCommand("BE", String(ms < 1 ? 100 : (ms > 1000 ? 1000 : ms))); }

void McSerialClient::requestPosition() { sendCommand("IP"); }

void McSerialClient::startPathStream() {
  if (path_ == nullptr || path_->empty()) {
    return;
  }
  sendCommand("PC");
  pdStreamIndex_ = 0;
  pdStreaming_ = true;
}

void McSerialClient::setPathSliceUs(uint32_t us) {
  if (us < 1000) {
    us = 1000;
  }
  pathSliceUs_ = us;
  sendCommand("PS", String(us));
}

void McSerialClient::moveToFramePose(int dfFrame) {
  if (path_ == nullptr) {
    return;
  }
  int local = 0;
  if (!path_->localFrame(dfFrame, &local)) {
    return;
  }
  float mm[kMaxAxes] = {};
  const int n = advertisedMotors();
  for (int a = 0; a < n; ++a) {
    mm[a] = dmcStepsToMc(path_->positionSteps(a, local));
    positionMm_[a] = mm[a];
    targetMm_[a] = mm[a];
  }
  currentFrame_ = dfFrame;
  sendCommand("MT", packMc(mm, n));
  movingMask_ = 1;
  lastMotionMs_ = millis();
}

void McSerialClient::pathGoRange(int dfStart, int dfEnd) {
  if (path_ == nullptr) {
    return;
  }
  int a = 1;
  int b = 1;
  if (!path_->sampleRangeForFrames(dfStart, dfEnd, &a, &b)) {
    sendCommand("PG");
  } else {
    sendCommand("PG", String(a) + " " + String(b));
  }
  playStartFrame_ = dfStart;
  playEndFrame_ = dfEnd;
  pathActive_ = true;
  movingMask_ = 1;
  lastMotionMs_ = millis();
  if (simulated_) {
    if (!path_->sampleRangeForFrames(dfStart, dfEnd, &simPgA_, &simPgB_)) {
      simPgA_ = 0;
      simPgB_ = path_->sampleCount() > 0 ? path_->sampleCount() - 1 : 0;
    }
    simPathSample_ = simPgA_;
    simPathNextMs_ = millis();
  }
}

bool McSerialClient::hardStopLatched() {
  const bool v = hardStop_;
  hardStop_ = false;
  return v;
}

int32_t McSerialClient::positionSteps(int axis0) const {
  if (axis0 < 0 || axis0 >= kMaxAxes) {
    return 0;
  }
  return mcToDmcSteps(positionMm_[axis0]);
}

void McSerialClient::update() {
  while (Serial1.available()) {
    const int c = Serial1.read();
    if (c < 0) {
      break;
    }
    const char ch = static_cast<char>(c);
    if (ch == '\n' || ch == '\r') {
      if (rxBuffer_.length() > 0) {
        handleMcLine(rxBuffer_);
        rxBuffer_ = "";
      }
    } else {
      rxBuffer_ += ch;
      if (rxBuffer_.length() > 512) {
        rxBuffer_ = rxBuffer_.substring(rxBuffer_.length() - 256);
      }
    }
  }

  if (configDrain_ && millis() - lastCgMs_ >= kMcConfigIdleMs) {
    configDrain_ = false;
    applyLiveSettings();
  }

  if (!startupChecked_ && millis() >= startupDeadlineMs_) {
    startupChecked_ = true;
    if (!mcPresent_ && kSimulateMc) {
      startSimulation();
    }
  }

  pumpPathStream();
  updateSimMotion();
  if (simulated_ && pathActive_) {
    updateSimPath();
  }
}

void McSerialClient::parsePackedFloats(const String& body, float* out, int n) {
  int idx = 0;
  int start = 0;
  while (idx < n && start <= body.length()) {
    int pipe = body.indexOf('|', start);
    String tok = (pipe < 0) ? body.substring(start) : body.substring(start, pipe);
    tok.trim();
    if (tok.length() > 0) {
      out[idx] = tok.toFloat();
    }
    ++idx;
    if (pipe < 0) {
      break;
    }
    start = pipe + 1;
  }
}

void McSerialClient::markPresent() {
  mcPresent_ = true;
  startupChecked_ = true;
  configDrain_ = true;
  lastCgMs_ = millis();
}

void McSerialClient::applyLiveSettings() {
  if (liveApplied_ || simulated_) {
    return;
  }
  liveApplied_ = true;
  sendCommand("SE", "1");
  sendCommand("SV", "1");
  sendCommand("CS", "verbose_rate_hz 10");
}

void McSerialClient::startSimulation() {
  simulated_ = true;
  mcPresent_ = true;
  liveApplied_ = true;
  config_.axisCount = 1;
  config_.motorCount = 1;
  config_.servoCount = 0;
  config_.maxSpeed[0] = 100.0f;
  config_.maxAccel[0] = 50.0f;
  config_.initSpeed = 40.0f;
  config_.initAccel = 20.0f;
  sessionSpeed_ = 40.0f;
  sessionAccel_ = 20.0f;
  lastMotionMs_ = millis();
  debugLog("MC simulator started with 1 axis");
}

void McSerialClient::simulateCgDump() {
  handleMcLine("# MC V1 - Slider Motion Controller - 1+0 axis ['?' for help]");
  handleMcLine("CG:axis=1");
  handleMcLine("CG:motor_count=1");
  handleMcLine("CG:servo_count=0");
  handleMcLine("CG:motor_1_max_speed=100");
  handleMcLine("CG:motor_1_max_accel=50");
  handleMcLine("CG:init_speed=40");
  handleMcLine("CG:init_accel=20");
  handleMcLine("CG:axis_1_unit=mm");
  handleMcLine("CG:motor_1_min=0");
  handleMcLine("CG:motor_1_max=1000");
  handleMcLine("CG:axis_min_1=0");
  handleMcLine("CG:axis_max_1=1000");
  handleMcLine("CG:motor_1_steps_per_unit=80");
}

void McSerialClient::simulateCommand(const String& line) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return;
  }
  if (trimmed == "VH") {
    handleMcLine("# MC V1 - Slider Motion Controller - 1+0 axis ['?' for help]");
    return;
  }
  if (trimmed == "CG") {
    simulateCgDump();
    return;
  }
  if (trimmed.startsWith("MT")) {
    parsePackedFloats(trimmed.substring(2), targetMm_, advertisedMotors());
    movingMask_ = 1;
    lastMotionMs_ = millis();
    handleMcLine("#M " + fmtMc(positionMm_[0]) + " 0.00 0.00");
    return;
  }
  if (trimmed.startsWith("MS")) {
    movingMask_ = 0;
    pathActive_ = false;
    memcpy(targetMm_, positionMm_, sizeof(targetMm_));
    handleMcLine("!E:STOPPED");
    handleMcLine("#I " + fmtMc(positionMm_[0]));
    return;
  }
  if (trimmed.startsWith("IP")) {
    handleMcLine("IP:" + packMc(positionMm_, advertisedMotors()));
    return;
  }
  if (trimmed.startsWith("SP")) {
    parsePackedFloats(trimmed.substring(2), positionMm_, advertisedMotors());
    memcpy(targetMm_, positionMm_, sizeof(targetMm_));
    movingMask_ = 0;
    handleMcLine("IP:" + packMc(positionMm_, advertisedMotors()));
    return;
  }
  if (trimmed.startsWith("SS")) {
    sessionSpeed_ = trimmed.substring(2).toFloat();
    if (sessionSpeed_ < 0.001f) {
      sessionSpeed_ = 0.001f;
    }
    return;
  }
  if (trimmed.startsWith("SA")) {
    sessionAccel_ = trimmed.substring(2).toFloat();
    return;
  }
  if (trimmed.startsWith("SL") || trimmed.startsWith("SR") || trimmed.startsWith("SE") ||
      trimmed.startsWith("SV") || trimmed.startsWith("CS") || trimmed.startsWith("CT") ||
      trimmed.startsWith("BE") || trimmed.startsWith("PC") || trimmed.startsWith("PS")) {
    return;
  }
  if (trimmed.startsWith("PD")) {
    return;
  }
  if (trimmed.startsWith("PG")) {
    pathActive_ = true;
    movingMask_ = 1;
    lastMotionMs_ = millis();
    handleMcLine("#P " + fmtMc(positionMm_[0]) + " 0.00 0.00");
    return;
  }
  if (trimmed == "PI" || trimmed.startsWith("PI ")) {
    handleMcLine("PI:" + String(simPathSample_));
    return;
  }
}

void McSerialClient::pumpPathStream() {
  if (!pdStreaming_ || path_ == nullptr) {
    return;
  }
  int batch = 6;
  while (batch-- > 0 && pdStreamIndex_ < path_->sampleCount()) {
    int16_t samp[kMaxAxes] = {};
    path_->fillPd(pdStreamIndex_, samp);
    String line = "PD ";
    const int n = path_->axisCount();
    for (int a = 0; a < n; ++a) {
      if (a > 0) {
        line += " | ";
      }
      line += String(static_cast<int>(samp[a]));
    }
    sendLine(line);
    ++pdStreamIndex_;
  }
  if (pdStreamIndex_ >= path_->sampleCount()) {
    pdStreaming_ = false;
  }
}

void McSerialClient::updateSimMotion() {
  if (!simulated_ || pathActive_ || movingMask_ == 0) {
    return;
  }
  const uint32_t now = millis();
  uint32_t elapsed = now - lastMotionMs_;
  if (elapsed == 0) {
    return;
  }
  lastMotionMs_ = now;
  float mm = sessionSpeed_ * (static_cast<float>(elapsed) / 1000.0f);
  if (mm < 0.0001f) {
    mm = 0.0001f;
  }
  bool any = false;
  const int n = advertisedMotors();
  for (int a = 0; a < n; ++a) {
    const float rem = targetMm_[a] - positionMm_[a];
    if (fabsf(rem) <= mm) {
      positionMm_[a] = targetMm_[a];
    } else {
      positionMm_[a] += (rem > 0) ? mm : -mm;
      any = true;
    }
  }
  if (!any) {
    movingMask_ = 0;
    handleMcLine("#I " + fmtMc(positionMm_[0]));
  }
}

void McSerialClient::updateSimPath() {
  if (path_ == nullptr || !pathActive_) {
    return;
  }
  const uint32_t now = millis();
  if (now < simPathNextMs_) {
    return;
  }
  simPathNextMs_ = now + (pathSliceUs_ / 1000);
  const int dir = (simPgA_ <= simPgB_) ? 1 : -1;
  const bool pastEnd = (dir > 0) ? (simPathSample_ > simPgB_) : (simPathSample_ < simPgB_);
  if (pastEnd || simPathSample_ < 0 || simPathSample_ >= path_->sampleCount()) {
    pathActive_ = false;
    movingMask_ = 0;
    currentFrame_ = playEndFrame_;
    handleMcLine("#I " + fmtMc(positionMm_[0]) + " 0.00 0.00");
    return;
  }
  int16_t samp[kMaxAxes] = {};
  path_->fillPd(simPathSample_, samp);
  const int n = advertisedMotors();
  const int sign = dir;
  for (int a = 0; a < n; ++a) {
    int32_t um = static_cast<int32_t>(samp[a]) * sign;
    positionMm_[a] += static_cast<float>(um) / 1000.0f;
    targetMm_[a] = positionMm_[a];
  }
  simPathSample_ += dir;
  movingMask_ = 1;
}

void McSerialClient::handleMcLine(const String& line) {
  String clean = line;
  clean.trim();
  if (clean.length() == 0) {
    return;
  }
  if (clean.startsWith("# Slider") || clean.startsWith("# MC")) {
    markPresent();
    return;
  }
  if (clean.startsWith("CG:")) {
    parseConfigLine(clean.substring(3));
    markPresent();
    return;
  }
  if (clean.startsWith("!E:")) {
    debugLog("MC error: ", clean.c_str());
    return;
  }
  if (clean.startsWith("IP:")) {
    parsePackedFloats(clean.substring(3), positionMm_, advertisedMotors());
    bool still = false;
    for (int a = 0; a < advertisedMotors(); ++a) {
      if (fabsf(positionMm_[a] - targetMm_[a]) > 0.0005f) {
        still = true;
      }
    }
    if (!still && !pathActive_) {
      movingMask_ = 0;
    }
    return;
  }
  if (clean.startsWith("IM:")) {
    const int v = clean.substring(3).toInt();
    if (v == 0 && !pathActive_) {
      movingMask_ = 0;
    } else if (v != 0) {
      movingMask_ = 1;
    }
    if (v == 0) {
      pathActive_ = false;
    }
    return;
  }
  if (clean.startsWith("IE:")) {
    if (clean.substring(3).toInt() != 0) {
      hardStop_ = true;
      movingMask_ = 0;
      pathActive_ = false;
    }
    return;
  }
  if (clean.startsWith("PI:")) {
    return;
  }
  if (clean.charAt(0) == '#' && clean.length() >= 2) {
    const char letter = clean.charAt(1);
    if (letter == 'E' || letter == 'L') {
      hardStop_ = true;
      movingMask_ = 0;
      pathActive_ = false;
    } else if (letter == 'P' || letter == 'M' || letter == 'A' || letter == 'B') {
      movingMask_ = 1;
      pathActive_ = (letter == 'P');
    } else if (letter == 'I' || letter == 'D') {
      movingMask_ = 0;
      pathActive_ = false;
    }
    writeMovePin(letter);
    const int sp = clean.indexOf(' ');
    if (sp > 0) {
      parsePackedFloats(clean.substring(sp + 1), positionMm_, 1);
    }
  }
}

void McSerialClient::parseConfigLine(const String& line) {
  String trimmed = line;
  trimmed.trim();
  const int eq = trimmed.indexOf('=');
  if (eq < 0) {
    return;
  }
  String key = trimmed.substring(0, eq);
  key.trim();
  String value = trimmed.substring(eq + 1);
  value.trim();
  auto axisIndex = [](const String& k, const char* prefix) -> int {
    if (!k.startsWith(prefix)) {
      return -1;
    }
    return k.substring(strlen(prefix)).toInt() - 1;
  };
  auto envFloat = [](const String& v) -> float {
    String t = v;
    t.trim();
    if (t.length() == 0) {
      return NAN;
    }
    String tl = t;
    tl.toLowerCase();
    if (tl == "none" || t == "-") {
      return NAN;
    }
    return v.toFloat();
  };
  auto motorField = [](const String& k, const char* field) -> int {
    if (!k.startsWith("motor_")) {
      return -1;
    }
    const int us = k.indexOf('_', 6);
    if (us < 0 || k.substring(us + 1) != field) {
      return -1;
    }
    return k.substring(6, us).toInt() - 1;
  };
  if (key == "axis") {
    config_.axisCount = value.toInt();
  } else if (key == "motor_count" || key == "motors") {
    config_.motorCount = value.toInt();
  } else if (key == "servo_count" || key == "servos") {
    config_.servoCount = value.toInt();
  } else if (key == "init_speed") {
    config_.initSpeed = value.toFloat();
  } else if (key == "init_accel") {
    config_.initAccel = value.toFloat();
  } else if (key.startsWith("axis_") && key.endsWith("_unit")) {
    const int a = key.substring(5, key.length() - 5).toInt() - 1;
    if (a >= 0 && a < kMaxAxes && value.length() > 0) {
      strncpy(config_.unitName[a], value.c_str(), sizeof(config_.unitName[a]) - 1);
      config_.unitName[a][sizeof(config_.unitName[a]) - 1] = 0;
    }
  } else {
    int a = motorField(key, "max_speed");
    if (a < 0) {
      a = axisIndex(key, "max_speed_");
    }
    if (a >= 0 && a < kMaxAxes) {
      config_.maxSpeed[a] = value.toFloat();
      return;
    }
    a = motorField(key, "max_accel");
    if (a < 0) {
      a = axisIndex(key, "max_accel_");
    }
    if (a >= 0 && a < kMaxAxes) {
      config_.maxAccel[a] = value.toFloat();
      return;
    }
    a = axisIndex(key, "axis_min_");
    if (a >= 0 && a < kMaxAxes) {
      config_.axisMin[a] = envFloat(value);
      return;
    }
    a = axisIndex(key, "axis_max_");
    if (a >= 0 && a < kMaxAxes) {
      config_.axisMax[a] = envFloat(value);
      return;
    }
    a = motorField(key, "steps_per_unit");
    if (a < 0) {
      a = axisIndex(key, "steps_per_unit_");
    }
    if (a >= 0 && a < kMaxAxes) {
      config_.stepsPerUnit[a] = value.toFloat();
      return;
    }
    a = motorField(key, "min");
    if (a >= 0 && a < kMaxAxes) {
      config_.axisMin[a] = envFloat(value);
      return;
    }
    a = motorField(key, "max");
    if (a >= 0 && a < kMaxAxes) {
      config_.axisMax[a] = envFloat(value);
      return;
    }
    if (key.startsWith("MOTOR_") && key.endsWith("_min")) {
      const int n = key.substring(6, key.indexOf('_', 6)).toInt() - 1;
      if (n >= 0 && n < kMaxAxes) {
        config_.axisMin[n] = envFloat(value);
      }
    } else if (key.startsWith("MOTOR_") && key.endsWith("_max")) {
      const int n = key.substring(6, key.indexOf('_', 6)).toInt() - 1;
      if (n >= 0 && n < kMaxAxes) {
        config_.axisMax[n] = envFloat(value);
      }
    }
  }
}

}  // namespace sliderdmc
