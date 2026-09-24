#include "bridge.h"

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
  std::vector<uint8_t> payload(32, 0);
  const size_t n = strlen(kDmcHelloName);
  std::memcpy(payload.data(), kDmcHelloName, n > 32 ? 32 : n);
  appendByte(payload, kHelloVersionMajor);
  appendByte(payload, kHelloVersionMinor);
  appendByte(payload, kHelloVersionRev);
  appendByte(payload, static_cast<uint8_t>(mcClient_.advertisedMotors()));
  appendWordLE(payload, static_cast<uint16_t>(kDmxChannels));
  appendByte(payload, 4);
  appendByte(payload, 4);
  appendByte(payload, 0);
  appendDwordLE(payload, static_cast<uint32_t>(kMaxUploadFrames));
  appendDwordLE(payload, kDmcCapRealTime | kDmcCapRealTimeCamera);
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
  if (wasPathActive_ && !active && dfConnected_) {
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  }
  wasPathActive_ = active;
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
    case kDmcMsgRtGo:
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
