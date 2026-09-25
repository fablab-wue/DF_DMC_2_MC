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
  maybeFinishPath();
  maybeFinishShoot();
  pumpPendingPlay();
  if (mcClient_.hardStopLatched()) {
    sendDmcAck(0, kDmcMsgMotorHardStop, kDmcAckOk);
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
  Serial.write(packet.data(), packet.size());
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

void DmcBridge::sendMotorPositions(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, 0);
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
  if (!dfConnected_ || !mcClient_.moving()) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastPositionTxMs_ < kPositionReportMs) {
    return;
  }
  lastPositionTxMs_ = now;
  sendMotorPositions(0);
}

void DmcBridge::maybeUnsolicitedGio() {
  if (dfConnected_ && gio_.pollInputChange()) {
    sendGioIn(0);
  }
}

void DmcBridge::maybeFinishPath() {
  const bool active = mcClient_.pathActive();
  if (wasPathActive_ && !active && dfConnected_ && !shootGoing_ && !shootPendingMf_) {
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  }
  wasPathActive_ = active;
}

void DmcBridge::maybeFinishShoot() {
  if (!shootGoing_ && !shootPendingMf_) {
    return;
  }
  if (mcClient_.takeCommandError()) {
    if (shootShutterOn_) {
      gio_.setCameraShutter(false);
      shootShutterOn_ = false;
    }
    shootGoing_ = false;
    shootPendingMf_ = false;
    shootArmed_ = false;
    return;
  }
  const uint32_t elapsed = millis() - shootT0_;
  if (shootPendingMf_ && elapsed >= shootDelayMs_) {
    if (!mcClient_.moveForMs(shootMoveMs_, shootRampMs_, shootEndSteps_, shootBlur_, mcClient_.advertisedMotors())) {
      shootPendingMf_ = false;
      shootGoing_ = false;
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
    shootGoing_ = false;
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
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
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
  mcClient_.moveToSteps(startSteps, n);
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::fireBloop(unsigned ms) {
  if (ms == 0) {
    ms = 100;
  }
  gio_.pulseBuzzer(ms);
  if (mcClient_.ready()) {
    mcClient_.beep(ms);
  }
}

void DmcBridge::applyFrameTrigger(int dfFrame) {
  int local = 0;
  if (path_.triggerMask() == 0 || !path_.localFrame(dfFrame, &local)) {
    return;
  }
  gio_.setOutputs(path_.triggerAtLocal(local));
}

void DmcBridge::pumpPendingPlay() {
  if (!pendingPlay_) {
    return;
  }
  if (!mcClient_.pathStreamDone()) {
    return;
  }
  if (millis() < pendingPlayAtMs_) {
    return;
  }
  pendingPlay_ = false;
  if (pendingBloopMs_ > 0) {
    fireBloop(pendingBloopMs_);
  }
  applyFrameTrigger(pendingStart_);
  mcClient_.pathGoRange(pendingStart_, pendingEnd_);
}

uint32_t DmcBridge::psFromFpsX1000(uint32_t fpsX1000) const {
  if (fpsX1000 < 1) {
    fpsX1000 = 24000;
  }
  uint32_t us = (1000000000UL + (fpsX1000 / 2)) / fpsX1000;
  if (us < 1000) {
    us = 1000;
  }
  return us;
}

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
      uint8_t ramp = 0;
      uint16_t channel = 1;
      uint16_t count = 0;
      const uint8_t* levels = nullptr;
      bool parsed = false;
      if (frame.payload.size() >= 7) {
        uint32_t channel32 = 0;
        uint16_t counted = 0;
        uint8_t rampByte = 0;
        if (readDwordLE(frame.payload, 0, &channel32) && readWordLE(frame.payload, 4, &counted) &&
            readByte(frame.payload, 6, &rampByte) && counted > 0 && 7u + counted == frame.payload.size() &&
            channel32 >= 1 && channel32 <= static_cast<uint32_t>(kDmxChannels)) {
          channel = static_cast<uint16_t>(channel32);
          count = counted;
          ramp = rampByte;
          levels = frame.payload.data() + 7;
          parsed = true;
        }
      }
      if (!parsed) {
        if (frame.payload.size() < 4 || !readByte(frame.payload, 0, &ramp) || !readWordLE(frame.payload, 1, &channel)) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
          break;
        }
        count = static_cast<uint16_t>(frame.payload.size() - 3);
        levels = frame.payload.data() + 3;
      }
      if (channel < 1 || channel > kDmxChannels || count == 0) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      dmx_.apply(channel, levels, count, ramp != 0);
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
      sendMotorPositions(frame.id);
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
      mcClient_.stopMotion();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorStopAll:
    case kDmcMsgMotorHardStop:
      mcClient_.stopMotion();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
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
      (void)speed;
      mcClient_.moveAxisToSteps(motor - 1, destination);
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
    case kDmcMsgRtUploadDmx:
      sendDmcAck(frame.id, frame.type, kDmcAckErrUnsupported);
      break;
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
      mcClient_.moveToFramePose(frameNo);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      lastPositionTxMs_ = millis();
      break;
    }
    case kDmcMsgRtRunMove: {
      uint32_t fpsX1000 = 24000;
      int32_t startFrame = 1;
      int32_t endFrame = 1;
      uint32_t prerollMs = 0;
      uint32_t postrollMs = 0;
      uint8_t syncDmx = 0;
      uint32_t bloopLoc = 0;
      uint16_t bloopDmx = 0;
      uint16_t bloopTime = 0;
      if (frame.payload.size() < 20 || !readDwordLE(frame.payload, 0, &fpsX1000) ||
          !readSignedDwordLE(frame.payload, 4, &startFrame) || !readSignedDwordLE(frame.payload, 8, &endFrame)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (frame.payload.size() >= 24) {
        readDwordLE(frame.payload, 12, &prerollMs);
      }
      if (frame.payload.size() >= 28) {
        readDwordLE(frame.payload, 16, &postrollMs);
      }
      (void)postrollMs;
      if (frame.payload.size() >= 29) {
        readByte(frame.payload, 20, &syncDmx);
      }
      (void)syncDmx;
      if (frame.payload.size() >= 33) {
        readDwordLE(frame.payload, 21, &bloopLoc);
      }
      if (frame.payload.size() >= 35) {
        readWordLE(frame.payload, 25, &bloopDmx);
      }
      (void)bloopDmx;
      if (frame.payload.size() >= 37) {
        readWordLE(frame.payload, 27, &bloopTime);
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      mcClient_.setPathSliceUs(psFromFpsX1000(fpsX1000));
      mcClient_.moveToFramePose(startFrame);
      pendingStart_ = startFrame;
      pendingEnd_ = endFrame;
      pendingBloopMs_ = (bloopLoc != 0 || bloopTime != 0) ? (bloopTime == 0 ? 100 : bloopTime) : 0;
      pendingPrerollMs_ = prerollMs;
      pendingPlayAtMs_ = millis() + prerollMs;
      pendingPlay_ = true;
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
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      pendingStart_ = path_.startFrame();
      pendingEnd_ = path_.endFrame();
      pendingBloopMs_ = 0;
      pendingPrerollMs_ = 0;
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
      pendingBloopMs_ = 0;
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
