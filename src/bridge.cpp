#include "bridge.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace sliderdmc {

void DmcBridge::setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  const GioMap gioMap{kGioOutPins, 4, kGioInPins, 4, kCameraShutterPin, kBuzzerPin, kMovePin};
  gio_.begin(gioMap);
  dmx_.begin(kDmxTxPin, kDmxPwmPins, 6, kDmxPwmHz, false);
  dfConnected_ = false;
  bootHelloSent_ = false;
  mcClient_.attachPath(&path_);
  mcClient_.begin();
  statusLed_.begin();
}

void DmcBridge::loop() {
  noteCdc(static_cast<bool>(Serial));
  maybeSendBootHello();
  while (Serial.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    DmcFrame frame;
    if (dmcParser_.feed(byte, &frame)) {
      statusLed_.markActivity();
      handleDmcFrame(frame);
    }
  }
  mcClient_.update();
  dmx_.update();
  gio_.tick();
  maybeSendPositionReport();
  maybeUnsolicitedGio();
  bloop_.tick(gio_, dmx_);
  shutter_.tick(gio_);
  if (mcClient_.hardStopLatched()) {
    sendHardStop(0);
    cancelLive(true);
  }
  maybeFinishPath();
  maybeFinishShoot();
  pumpPendingPlay();
  if (mcClient_.takeFrameEdge()) {
    const int frame = mcClient_.currentFrame();
    if (syncDmx_) {
      applyProgramDmx(frame);
    }
    applyFrameTrigger(frame);
    if (inRun(frame)) {
      shutter_.beginFrame(gio_);
    }
    sendMotorPositions(0, static_cast<int32_t>(moveTimeThousandths(frame)));
  }
  statusLed_.update(dfConnected_, mcClient_.mcPresent(), mcClient_.simulatorActive(),
                    mcClient_.startupTimedOut());
}

bool DmcBridge::motorIndexValid(uint8_t motor) const {
  return motor >= 1 && motor <= static_cast<uint8_t>(mcClient_.advertisedMotors());
}

void DmcBridge::sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> packet;
  packet.reserve(kDmcHeaderSize + payload.size() + kDmcCsumSize);
  packet.push_back('D');
  packet.push_back('F');
  appendDwordLE(packet, id);
  appendWordLE(packet, type);
  appendWordLE(packet, static_cast<uint16_t>(payload.size()));
  packet.insert(packet.end(), payload.begin(), payload.end());
  const uint16_t rawChecksum = computeChecksum(packet.data(), packet.size());
  appendWordLE(packet, encodeChecksum(rawChecksum));
  writeUsbFrame(packet.data(), packet.size());
}

void DmcBridge::sendDmcAck(uint32_t id, uint16_t type, uint32_t status) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, status);
  sendDmcFrame(id, static_cast<uint16_t>(type | kDmcMsgFlagAck), payload);
}

void DmcBridge::sendDmcHello(uint32_t id) {
  const McConfig& cfg = mcClient_.config();
  char name[48];
  const int wrote = snprintf(name, sizeof(name), "jDF-MC V1 %dM+%dS+6L+4O+4I+CT+DMX", cfg.motorCount, cfg.servoCount);
  std::vector<uint8_t> payload(32, 0);
  if (wrote > 0) {
    const size_t n = static_cast<size_t>(wrote) > 32 ? 32 : static_cast<size_t>(wrote);
    std::memcpy(payload.data(), name, n);
  }
  appendByte(payload, kHelloVersionMajor);
  appendByte(payload, kHelloVersionMinor);
  appendByte(payload, kHelloVersionRev);
  appendByte(payload, static_cast<uint8_t>(mcClient_.advertisedMotors()));
  appendWordLE(payload, static_cast<uint16_t>(kDmxChannels));
  appendByte(payload, 4);
  appendByte(payload, 4);
  appendByte(payload, 0);
  appendDwordLE(payload, static_cast<uint32_t>(kMaxUploadFrames));
  appendDwordLE(payload, kDmcCapRealTime | kDmcCapGoMotion | kDmcCapGoMotion2 | kDmcCapRealTimeCamera);
  appendWordLE(payload, kHelloProtocolVersion);
  sendDmcFrame(id, kDmcMsgHi, payload);
}

void DmcBridge::sendMotorStatus(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, mcClient_.movingMask());
  appendByte(payload, dmx_.ramping() ? 1 : 0);
  sendDmcFrame(id, kDmcMsgMotorStatus, payload);
}

void DmcBridge::sendMotorPositions(uint32_t id, int32_t frameTime) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, static_cast<uint32_t>(frameTime));
  const int n = mcClient_.advertisedMotors();
  for (int a = 0; a < n; ++a) {
    appendDwordLE(payload, static_cast<uint32_t>(mcClient_.positionSteps(a)));
  }
  sendDmcFrame(id, kDmcMsgMotorGetPosition, payload);
}

void DmcBridge::sendGioIn(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, gio_.inputs());
  sendDmcFrame(id, kDmcMsgGioIn, payload);
}

void DmcBridge::maybeSendBootHello() {
  const bool cdcConnected = static_cast<bool>(Serial);
  if (cdcConnected && !bootHelloSent_) {
    sendDmcHello(0);
    bootHelloSent_ = true;
  } else if (!cdcConnected) {
    bootHelloSent_ = false;
  }
}

void DmcBridge::maybeSendPositionReport() {
  if (!dfConnected_ || !mcClient_.moving() || mcClient_.pathActive()) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastPositionTxMs_ < kPositionReportMs) {
    return;
  }
  lastPositionTxMs_ = now;
  if (postrollWaiting_) {
    sendMotorPositions(0, static_cast<int32_t>(moveTimeThousandths(static_cast<int32_t>(lround(postrollFrame_)))));
  } else {
    sendMotorPositions(0);
  }
}

void DmcBridge::maybeUnsolicitedGio() {
  if (dfConnected_ && gio_.pollInputChange()) {
    sendGioIn(0);
  }
}

void DmcBridge::noteCdc(bool up) {
  if (cdcWasUp_ && !up) {
    dfConnected_ = false;
    bootHelloSent_ = false;
    dmcParser_.reset();
    cancelLive(false);
  }
  cdcWasUp_ = up;
}

void DmcBridge::sendHardStop(uint8_t reason) {
  std::vector<uint8_t> payload;
  appendByte(payload, reason);
  sendDmcFrame(0, kDmcMsgMotorHardStop, payload);
}

void DmcBridge::endShoot(bool notify) {
  const bool pending = shootNeedsEnd_;
  if (shootShutterOn_) {
    gio_.setCameraShutter(false);
    shootShutterOn_ = false;
  }
  shootArmed_ = false;
  shootGoing_ = false;
  shootPendingMf_ = false;
  shootNeedsEnd_ = false;
  if (notify && pending && dfConnected_) {
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  }
}

void DmcBridge::cancelLive(bool notify) {
  const bool playback = wasPathActive_ || postrollWaiting_ || pendingPlay_ || armedPlay_ || mcClient_.pathActive();
  mcClient_.stopMotion();
  mcClient_.takeCommandError();
  armedPlay_ = false;
  pendingPlay_ = false;
  postrollWaiting_ = false;
  postrollMoveSent_ = false;
  pendingPostrollMs_ = 0;
  shutter_.endMove(gio_);
  bloop_.release(gio_, dmx_);
  if (shootNeedsEnd_) {
    endShoot(notify);
    wasPathActive_ = false;
  } else if (notify && playback && dfConnected_) {
    wasPathActive_ = false;
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  } else {
    wasPathActive_ = false;
  }
}

bool DmcBridge::moveToSample(double frameTime) {
  if (mcClient_.pathActive()) {
    mcClient_.stopMotion();
    mcClient_.takeCommandError();
  }
  const int n = mcClient_.advertisedMotors();
  int32_t steps[kMaxAxes] = {};
  bool differ = false;
  for (int a = 0; a < n; ++a) {
    steps[a] = path_.sampleSteps(a, frameTime, true);
    if (steps[a] != mcClient_.positionSteps(a)) {
      differ = true;
    }
  }
  mcClient_.setCurrentFrame(static_cast<int>(lround(frameTime)));
  if (differ) {
    mcClient_.moveToSteps(steps, n);
  }
  return differ;
}

uint32_t DmcBridge::poseFault(double frameTime, bool extrapolate) const {
  const int n = mcClient_.advertisedMotors();
  for (int a = 0; a < n; ++a) {
    const int fault = mcClient_.limitFault(a, path_.sampleSteps(a, frameTime, extrapolate));
    if (fault != 0) {
      return static_cast<uint32_t>(fault);
    }
  }
  return 0;
}

bool DmcBridge::rejectRunLimits(const RtRunMove& move, const RtPlaySpan& span, uint32_t id) {
  if (poseFault(span.prerollFrame, true) != 0) {
    sendDmcAck(id, kDmcMsgRtRunMove, kDmcAckErrPreroll);
    return true;
  }
  if (poseFault(span.postrollFrame, true) != 0) {
    sendDmcAck(id, kDmcMsgRtRunMove, kDmcAckErrPostroll);
    return true;
  }
  const int lo = move.startFrame < move.endFrame ? move.startFrame : move.endFrame;
  const int hi = move.startFrame < move.endFrame ? move.endFrame : move.startFrame;
  const int from = lo > path_.startFrame() ? lo : path_.startFrame();
  const int to = hi < path_.endFrame() ? hi : path_.endFrame();
  for (int f = from; f <= to; ++f) {
    const uint32_t fault = poseFault(static_cast<double>(f), false);
    if (fault != 0) {
      sendDmcAck(id, kDmcMsgRtRunMove, fault);
      return true;
    }
  }
  return false;
}

bool DmcBridge::inRun(int frame) const {
  const int lo = runStart_ < runEnd_ ? runStart_ : runEnd_;
  const int hi = runStart_ < runEnd_ ? runEnd_ : runStart_;
  return frame >= lo && frame <= hi;
}

void DmcBridge::maybeFinishPath() {
  const bool active = mcClient_.pathActive();
  const bool shoot = shootGoing_ || shootPendingMf_ || shootArmed_;
  if (active) {
    postrollWaiting_ = false;
  } else if (wasPathActive_ && !shoot && !postrollWaiting_) {
    postrollMoveSent_ = moveToSample(postrollFrame_);
    postrollUntilMs_ = millis() + pendingPostrollMs_;
    pendingPostrollMs_ = 0;
    postrollWaiting_ = true;
  } else if (postrollWaiting_ && !mcClient_.pathActive() && static_cast<int32_t>(millis() - postrollUntilMs_) >= 0 &&
             (!postrollMoveSent_ || !mcClient_.moving())) {
    postrollWaiting_ = false;
    postrollMoveSent_ = false;
    if (dfConnected_ && !shoot) {
      shutter_.endMove(gio_);
      bloop_.release(gio_, dmx_);
      sendDmcFrame(0, kDmcMsgRtEnd, {});
    }
  }
  wasPathActive_ = active;
}

void DmcBridge::maybeFinishShoot() {
  if (!shootGoing_ && !shootPendingMf_) {
    return;
  }
  if (mcClient_.takeCommandError()) {
    endShoot(true);
    wasPathActive_ = false;
    postrollWaiting_ = false;
    postrollMoveSent_ = false;
    return;
  }
  const uint32_t elapsed = millis() - shootT0_;
  if (shootPendingMf_ && elapsed >= shootDelayMs_) {
    if (!mcClient_.moveForMs(shootMoveMs_, shootRampMs_, shootEndSteps_, shootBlur_, mcClient_.advertisedMotors())) {
      endShoot(true);
      wasPathActive_ = false;
      postrollWaiting_ = false;
      postrollMoveSent_ = false;
      return;
    }
    shootPendingMf_ = false;
    shootGoing_ = true;
  }
  if (!shootGoing_) {
    return;
  }
  if (!shootShutterOn_ && elapsed >= shootShutterOpenMs_) {
    gio_.setCameraShutter(true);
    shootShutterOn_ = true;
  }
  if (shootShutterOn_ && elapsed >= shootShutterCloseMs_) {
    gio_.setCameraShutter(false);
    shootShutterOn_ = false;
  }
  if (elapsed >= shootDoneMs_ && !mcClient_.moving()) {
    endShoot(true);
  }
}

static int32_t sampleSteps(const PathStore& path, int axis, double frameTime) {
  const int lo = path.startFrame() < path.endFrame() ? path.startFrame() : path.endFrame();
  const int hi = path.startFrame() > path.endFrame() ? path.startFrame() : path.endFrame();
  if (frameTime < lo) {
    frameTime = lo;
  }
  if (frameTime > hi) {
    frameTime = hi;
  }
  const int f0 = static_cast<int>(floor(frameTime));
  int f1 = f0 + 1;
  if (f1 > hi) {
    f1 = hi;
  }
  int local0 = 0;
  if (!path.localFrame(f0, &local0)) {
    return 0;
  }
  const int32_t p0 = path.positionSteps(axis, local0);
  if (f1 == f0) {
    return p0;
  }
  int local1 = 0;
  if (!path.localFrame(f1, &local1)) {
    return p0;
  }
  const double u = frameTime - static_cast<double>(f0);
  return p0 + static_cast<int32_t>(lround((path.positionSteps(axis, local1) - p0) * u));
}

void DmcBridge::handleShootFrame(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint8_t direction = 1;
  uint32_t exposureMs = 0;
  uint16_t blurX10 = 0;
  if (frame.payload.size() < 11 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readByte(frame.payload, 4, &direction) || !readDwordLE(frame.payload, 5, &exposureMs) ||
      !readWordLE(frame.payload, 9, &blurX10)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (mcClient_.moving() || shootGoing_ || shootPendingMf_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (blurX10 == 0) {
    blurX10 = 1000;
  }

  const int n = mcClient_.advertisedMotors();
  bool overrideAxis[kMaxAxes] = {};
  int32_t posA[kMaxAxes] = {};
  int32_t posB[kMaxAxes] = {};
  size_t off = 11;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }

  const int dirSign = direction ? 1 : -1;
  const double dt = static_cast<double>(blurX10) * 0.0005;
  const double te = static_cast<double>(exposureMs) / 1000.0;
  int32_t startSteps[kMaxAxes] = {};
  for (int axis = 0; axis < n; ++axis) {
    int local = 0;
    int32_t framePose = mcClient_.positionSteps(axis);
    if (path_.localFrame(dfFrame, &local)) {
      framePose = path_.positionSteps(axis, local);
    }
    int32_t openPose = framePose;
    int32_t closePose = framePose;
    if (mcClient_.blurEnabled(axis) && overrideAxis[axis]) {
      const double delta = (0.5 - fabs(dt)) * static_cast<double>(posB[axis] - posA[axis]);
      openPose = posA[axis] + static_cast<int32_t>(lround(delta));
      closePose = posB[axis] - static_cast<int32_t>(lround(delta));
    } else if (mcClient_.blurEnabled(axis) && !path_.empty()) {
      openPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - dirSign * dt);
      closePose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + dirSign * dt);
    }
    const int32_t span = closePose - openPose;
    const int sign = span >= 0 ? 1 : -1;
    const double v = te > 0.0 ? fabs(static_cast<double>(span)) / te : 0.0;
    const int32_t accel = static_cast<int32_t>(lround(0.5 * v));
    shootBlur_[axis] = span != 0;
    startSteps[axis] = shootBlur_[axis] ? openPose - sign * accel : framePose;
    shootEndSteps_[axis] = shootBlur_[axis] ? closePose + sign * accel : framePose;
  }
  for (int axis = n; axis < kMaxAxes; ++axis) {
    shootBlur_[axis] = false;
  }

  shootMoveMs_ = exposureMs + 2000;
  shootRampMs_ = 1000;
  shootDelayMs_ = 0;
  shootShutterOpenMs_ = 1000;
  shootShutterCloseMs_ = 1000 + exposureMs;
  shootDoneMs_ = 2000 + exposureMs;
  shootGoing_ = false;
  shootShutterOn_ = false;
  shootArmed_ = true;
  shootNeedsEnd_ = true;
  mcClient_.moveToSteps(startSteps, n);
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::handleShootFrame2(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint32_t exposureMs = 0;
  uint16_t openWord = 0;
  uint16_t closeWord = 0;
  if (frame.payload.size() < 12 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readDwordLE(frame.payload, 4, &exposureMs) || !readWordLE(frame.payload, 8, &openWord) ||
      !readWordLE(frame.payload, 10, &closeWord)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (mcClient_.moving() || shootGoing_ || shootPendingMf_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (exposureMs == 0) {
    exposureMs = 1000;
  }
  if (exposureMs > 60000) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  const int16_t shutterOpen = static_cast<int16_t>(openWord);
  const int16_t shutterClose = static_cast<int16_t>(closeWord);
  if (shutterClose <= shutterOpen) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }

  const int n = mcClient_.advertisedMotors();
  bool overrideAxis[kMaxAxes] = {};
  int32_t posA[kMaxAxes] = {};
  int32_t posB[kMaxAxes] = {};
  size_t off = 12;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }

  const double te = static_cast<double>(exposureMs) / 1000.0;
  const double degrees = static_cast<double>(shutterClose - shutterOpen);
  const double secondsPerDegree = te / degrees;
  const double moveT = 360.0 * secondsPerDegree;
  const uint32_t moveMs = static_cast<uint32_t>(lround(moveT * 1000.0));
  const uint32_t rampMs = static_cast<uint32_t>(lround(moveT * 0.125 * 1000.0));
  if (moveMs < 1 || moveMs > 60000 || rampMs < 1 || rampMs * 2 >= moveMs) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  uint32_t delayMs = 0;
  uint32_t shutterOpenMs = 0;
  if (shutterOpen < 0) {
    delayMs = static_cast<uint32_t>(lround(-shutterOpen * secondsPerDegree * 1000.0));
    shutterOpenMs = 0;
  } else {
    shutterOpenMs = static_cast<uint32_t>(lround(shutterOpen * secondsPerDegree * 1000.0));
  }
  const uint32_t shutterCloseMs = shutterOpenMs + exposureMs;
  uint32_t postMs = 0;
  if (shutterClose > 360) {
    postMs = static_cast<uint32_t>(lround((shutterClose - 360) * secondsPerDegree * 1000.0));
  }

  int32_t startSteps[kMaxAxes] = {};
  for (int axis = 0; axis < n; ++axis) {
    int local = 0;
    int32_t framePose = mcClient_.positionSteps(axis);
    if (path_.localFrame(dfFrame, &local)) {
      framePose = path_.positionSteps(axis, local);
    }
    int32_t startPose = framePose;
    int32_t endPose = framePose;
    if (mcClient_.blurEnabled(axis) && overrideAxis[axis]) {
      startPose = posA[axis];
      endPose = posB[axis];
    } else if (mcClient_.blurEnabled(axis) && !path_.empty()) {
      startPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - 0.5);
      endPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + 0.5);
    }
    shootBlur_[axis] = startPose != endPose;
    startSteps[axis] = shootBlur_[axis] ? startPose : framePose;
    shootEndSteps_[axis] = shootBlur_[axis] ? endPose : framePose;
  }
  for (int axis = n; axis < kMaxAxes; ++axis) {
    shootBlur_[axis] = false;
  }

  shootMoveMs_ = moveMs;
  shootRampMs_ = rampMs;
  shootDelayMs_ = delayMs;
  shootShutterOpenMs_ = shutterOpenMs;
  shootShutterCloseMs_ = shutterCloseMs;
  shootDoneMs_ = delayMs + moveMs + postMs;
  shootGoing_ = false;
  shootShutterOn_ = false;
  shootArmed_ = true;
  shootNeedsEnd_ = true;
  mcClient_.moveToSteps(startSteps, n);
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::applyProgramDmx(int dfFrame) {
  int local = 0;
  if (!path_.localFrame(dfFrame, &local)) {
    return;
  }
  for (int slot = 0; slot < path_.dmxSlotCount(); ++slot) {
    const uint16_t channel = path_.dmxChannel(slot);
    const uint8_t level = path_.dmxLevel(slot, local);
    if (channel >= 1) {
      dmx_.apply(channel, &level, 1, false);
    }
  }
}

void DmcBridge::applyFrameTrigger(int dfFrame) {
  int local = 0;
  if (path_.triggerMask() == 0 || !path_.localFrame(dfFrame, &local)) {
    return;
  }
  gio_.setOutputs(path_.triggerAtLocal(local) | bloop_.heldOutputs());
}

void DmcBridge::pumpPendingPlay() {
  if (!pendingPlay_) {
    return;
  }
  if (!mcClient_.pathStreamDone() || mcClient_.moving()) {
    return;
  }
  if (millis() < pendingPlayAtMs_) {
    return;
  }
  pendingPlay_ = false;
  applyFrameTrigger(pendingStart_);
  if (syncDmx_) {
    applyProgramDmx(pendingStart_);
  }
  bloop_.fire(gio_, dmx_);
  if (bloop_.running() && mcClient_.ready()) {
    mcClient_.beep(bloop_.holdMs());
  }
  shutter_.beginMove(gio_);
  mcClient_.pathGoRange(pendingStart_, pendingEnd_);
}

uint32_t DmcBridge::psFromFpsX1000(uint32_t fpsX1000) const { return framePeriodUs(fpsX1000); }

void DmcBridge::handleDmcFrame(const DmcFrame& frame) {
  if (!frame.valid) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrChecksum);
    return;
  }

  if (frame.type == kDmcMsgHi) {
    dfConnected_ = true;
    sendDmcHello(frame.id);
    return;
  }

  switch (frame.type) {
    case kDmcMsgGioOut: {
      uint32_t bits = 0;
      if (!readDwordLE(frame.payload, 0, &bits)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      gio_.setOutputs(bits);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgGioIn:
      sendGioIn(frame.id);
      break;
    case kDmcMsgGioCam: {
      uint32_t cam = 0;
      if (!readDwordLE(frame.payload, 0, &cam)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool shutter = (cam & kDmcGioCamShutter) != 0;
      gio_.setCameraShutter(shutter);
      if (shutter && mcClient_.ready()) {
        mcClient_.cameraTrigger(100);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgDmx: {
      LiveDmx live;
      const LiveDmxStatus st = parseLiveDmx(frame.payload, &live);
      if (st == LiveDmxStatus::kGeneral) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (st == LiveDmxStatus::kRange) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      dmx_.apply(live.channel, live.levels, live.count, live.ramp);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorStatus:
      sendMotorStatus(frame.id);
      break;
    case kDmcMsgMotorGetPosition:
      if (mcClient_.ready()) {
        mcClient_.requestPosition();
      }
      if (mcClient_.pathActive()) {
        sendMotorPositions(frame.id, static_cast<int32_t>(moveTimeThousandths(mcClient_.currentFrame())));
      } else if (postrollWaiting_) {
        sendMotorPositions(frame.id,
                           static_cast<int32_t>(moveTimeThousandths(static_cast<int32_t>(lround(postrollFrame_)))));
      } else {
        sendMotorPositions(frame.id);
      }
      break;
    default:
      handleMotorOrRt(frame);
      break;
  }
}

void DmcBridge::handleMotorOrRt(const DmcFrame& frame) {
  if (!mcClient_.ready() && frame.type != kDmcMsgRtUploadBegin && frame.type != kDmcMsgRtUploadAxis &&
      frame.type != kDmcMsgRtUploadEnd && frame.type != kDmcMsgRtUploadTriggers) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }

  switch (frame.type) {
    case kDmcMsgMotorMove: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const int fault = mcClient_.limitFault(motor - 1, position);
      if (fault != 0) {
        sendDmcAck(frame.id, frame.type, static_cast<uint32_t>(fault));
        break;
      }
      if (shootNeedsEnd_ || wasPathActive_ || postrollWaiting_ || pendingPlay_ || armedPlay_ || mcClient_.pathActive()) {
        cancelLive(true);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      if (position == mcClient_.positionSteps(motor - 1) && !mcClient_.moving()) {
        sendMotorPositions(frame.id);
      } else {
        mcClient_.moveAxisToSteps(motor - 1, position);
        lastPositionTxMs_ = millis();
      }
      break;
    }
    case kDmcMsgMotorStop: {
      uint8_t motor = 0;
      if (!readByte(frame.payload, 0, &motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (shootNeedsEnd_ || wasPathActive_ || postrollWaiting_ || pendingPlay_ || armedPlay_ || mcClient_.pathActive()) {
        cancelLive(true);
      } else {
        mcClient_.stopAxis(motor - 1);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorStopAll:
    case kDmcMsgMotorHardStop: {
      const uint32_t now = millis();
      const bool hard = frame.type == kDmcMsgMotorHardStop ||
                        (lastStopAllMs_ != 0 && static_cast<uint32_t>(now - lastStopAllMs_) < kDmcStopAllHardMs);
      if (frame.type == kDmcMsgMotorStopAll) {
        lastStopAllMs_ = now;
      }
      cancelLive(true);
      if (hard) {
        mcClient_.stopMotion();
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorResetPosition: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (mcClient_.moving()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
        break;
      }
      mcClient_.resetAxisSteps(motor - 1, position);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorJog: {
      uint8_t motor = 0;
      uint16_t speed = 0;
      int32_t destination = 0;
      if (frame.payload.size() != 7 || !readByte(frame.payload, 0, &motor) || !readWordLE(frame.payload, 1, &speed) ||
          !readSignedDwordLE(frame.payload, 3, &destination)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const int fault = mcClient_.limitFault(motor - 1, destination);
      if (fault != 0) {
        sendDmcAck(frame.id, frame.type, static_cast<uint32_t>(fault));
        break;
      }
      if (shootNeedsEnd_ || wasPathActive_ || postrollWaiting_ || pendingPlay_ || armedPlay_ || mcClient_.pathActive()) {
        cancelLive(true);
      }
      mcClient_.jogAxisToSteps(motor - 1, destination, speed);
      lastPositionTxMs_ = millis();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorConfigure: {
      uint8_t motor = 0;
      uint8_t flags = 0;
      if (frame.payload.size() < 2 || !readByte(frame.payload, 0, &motor) || !readByte(frame.payload, 1, &flags)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      mcClient_.configure(motor - 1, flags);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetSpeed: {
      uint8_t motor = 0;
      int32_t maxVelocity = 0;
      int32_t maxAccel = 0;
      if (frame.payload.size() != 9 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &maxVelocity) || !readSignedDwordLE(frame.payload, 5, &maxAccel)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      mcClient_.setSpeedSteps(motor - 1, maxVelocity, maxAccel);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetLimits: {
      uint8_t motor = 0;
      uint8_t lowerEnable = 0;
      uint8_t upperEnable = 0;
      uint8_t hwSet = 0;
      int32_t lower = 0;
      int32_t upper = 0;
      if (frame.payload.size() < 12 || !readByte(frame.payload, 0, &motor) ||
          !readByte(frame.payload, 1, &lowerEnable) || !readSignedDwordLE(frame.payload, 2, &lower) ||
          !readByte(frame.payload, 6, &upperEnable) || !readSignedDwordLE(frame.payload, 7, &upper) ||
          !readByte(frame.payload, 11, &hwSet)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)hwSet;
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      mcClient_.setLimitsSteps(motor - 1, lowerEnable != 0, lower, upperEnable != 0, upper);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadBegin: {
      int32_t startFrame = 1;
      int32_t endFrame = 1;
      if (!readSignedDwordLE(frame.payload, 0, &startFrame) || !readSignedDwordLE(frame.payload, 4, &endFrame)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      path_.beginUpload(startFrame, endFrame, mcClient_.advertisedMotors());
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadAxis: {
      uint8_t motor = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 5 || !readByte(frame.payload, 0, &motor) || !readDwordLE(frame.payload, 1, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool finalFill = (index & kDmcDmxFlagFinalSet) != 0;
      index &= ~kDmcDmxFlagFinalSet;
      const int n = static_cast<int>((frame.payload.size() - 5) / 4);
      std::vector<int32_t> values(n);
      for (int i = 0; i < n; ++i) {
        readSignedDwordLE(frame.payload, 5 + static_cast<size_t>(i) * 4, &values[i]);
      }
      if (!path_.storeAxis(motor, index, values.data(), n, finalFill)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadDmx: {
      uint16_t channel = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 7 || !readWordLE(frame.payload, 0, &channel) || !readDwordLE(frame.payload, 2, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool finalFill = (index & kDmcDmxFlagFinalSet) != 0;
      index &= ~kDmcDmxFlagFinalSet;
      if (channel < 1 || channel > kDmxChannels) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const int n = static_cast<int>(frame.payload.size() - 6);
      if (!path_.storeDmx(channel, index, frame.payload.data() + 6, n, finalFill)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadTriggers: {
      uint32_t mask = 0;
      if (!readDwordLE(frame.payload, 0, &mask)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      size_t off = 4;
      while (off + 8 <= frame.payload.size()) {
        uint32_t idx = 0;
        uint32_t val = 0;
        readDwordLE(frame.payload, off, &idx);
        readDwordLE(frame.payload, off + 4, &val);
        path_.storeTrigger(mask, idx, val);
        off += 8;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadEnd:
      if (!path_.buildPd()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (mcClient_.ready()) {
        mcClient_.startPathStream();
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtPositionFrame: {
      int32_t frameNo = 0;
      if (!readSignedDwordLE(frame.payload, 0, &frameNo)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (shootNeedsEnd_ || wasPathActive_ || postrollWaiting_ || pendingPlay_ || armedPlay_ || mcClient_.pathActive()) {
        cancelLive(true);
      }
      mcClient_.moveToFramePose(frameNo);
      applyProgramDmx(frameNo);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      lastPositionTxMs_ = millis();
      sendMotorPositions(frame.id, static_cast<int32_t>(moveTimeThousandths(frameNo)));
      break;
    }
    case kDmcMsgRtRunMove: {
      RtRunMove move;
      if (!parseRtRunMove(frame.payload, &move)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      RtPlaySpan span;
      rtPlaySpan(move, &span);
      if (rejectRunLimits(move, span, frame.id)) {
        break;
      }
      wasPathActive_ = false;
      postrollWaiting_ = false;
      postrollMoveSent_ = false;
      pendingPlay_ = false;
      endShoot(true);
      mcClient_.setPathSliceUs(psFromFpsX1000(move.fpsX1000));
      moveToSample(span.prerollFrame);
      syncDmx_ = move.syncDmx;
      if (syncDmx_) {
        applyProgramDmx(move.startFrame);
      }
      runStart_ = move.startFrame;
      runEnd_ = move.endFrame;
      pendingStart_ = move.startFrame;
      pendingEnd_ = move.endFrame;
      postrollFrame_ = span.postrollFrame;
      pendingPostrollMs_ = move.postrollMs;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      bloop_.arm(move.bloopLocation, move.bloopDmx, move.bloopTimeMs);
      shutter_.arm(move.flags, move.cameraOpen, move.cameraClose, framePeriodUs(move.fpsX1000));
      armedPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtShootFrame:
      handleShootFrame(frame);
      break;
    case kDmcMsgRtShootFrame2:
      handleShootFrame2(frame);
      break;
    case kDmcMsgRtGo:
      if (shootArmed_) {
        if (mcClient_.moving() || shootGoing_ || shootPendingMf_) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrNotInPosition);
          break;
        }
        shootT0_ = millis();
        shootArmed_ = false;
        shootShutterOn_ = false;
        if (shootDelayMs_ == 0) {
          if (!mcClient_.moveForMs(shootMoveMs_, shootRampMs_, shootEndSteps_, shootBlur_,
                                   mcClient_.advertisedMotors())) {
            sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
            endShoot(true);
            break;
          }
          shootGoing_ = true;
          shootPendingMf_ = false;
        } else {
          shootPendingMf_ = true;
          shootGoing_ = false;
        }
        sendDmcAck(frame.id, frame.type, kDmcAckOk);
        break;
      }
      if (armedPlay_) {
        if (mcClient_.moving() || !mcClient_.pathStreamDone()) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrNotInPosition);
          break;
        }
        armedPlay_ = false;
        pendingPlayAtMs_ = millis();
        pendingPlay_ = true;
        sendDmcAck(frame.id, frame.type, kDmcAckOk);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      pendingStart_ = path_.startFrame();
      pendingEnd_ = path_.endFrame();
      runStart_ = pendingStart_;
      runEnd_ = pendingEnd_;
      postrollFrame_ = pendingEnd_;
      pendingPostrollMs_ = 0;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtJogAll: {
      uint32_t fpsX1000 = 24000;
      int32_t dest = 1;
      if (frame.payload.size() < 8 || !readDwordLE(frame.payload, 0, &fpsX1000) ||
          !readSignedDwordLE(frame.payload, 4, &dest)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      int from = mcClient_.currentFrame();
      if (from < path_.startFrame()) {
        from = path_.startFrame();
      }
      mcClient_.setPathSliceUs(psFromFpsX1000(fpsX1000));
      pendingStart_ = from;
      pendingEnd_ = dest;
      runStart_ = from;
      runEnd_ = dest;
      postrollFrame_ = dest;
      pendingPostrollMs_ = 0;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    default:
      sendDmcAck(frame.id, frame.type, kDmcAckErrUnsupported);
      break;
  }
}

}  // namespace sliderdmc
