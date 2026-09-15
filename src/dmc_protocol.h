#pragma once

// DragonFrame DMC v2 framing (USB CDC): DF magic, little-endian, Fletcher-16.

#include "config.h"

#include <array>
#include <cstdint>
#include <vector>

namespace sliderdmc {

constexpr uint8_t kDmcHeaderSize = 10;
constexpr uint8_t kDmcCsumSize = 2;
constexpr size_t kDmcMaxFrameSize = 2048;

constexpr uint16_t kDmcMsgFlagAck = 0x8000;
constexpr uint16_t kDmcMsgHi = 0x0001;
constexpr uint16_t kDmcMsgDmx = 0x0020;
constexpr uint16_t kDmcMsgGioOut = 0x0021;
constexpr uint16_t kDmcMsgGioIn = 0x0022;
constexpr uint16_t kDmcMsgGioCam = 0x0023;
constexpr uint16_t kDmcMsgMotorStatus = 0x0030;
constexpr uint16_t kDmcMsgMotorMove = 0x0031;
constexpr uint16_t kDmcMsgMotorStop = 0x0032;
constexpr uint16_t kDmcMsgMotorStopAll = 0x0033;
constexpr uint16_t kDmcMsgMotorGetPosition = 0x0034;
constexpr uint16_t kDmcMsgMotorResetPosition = 0x0035;
constexpr uint16_t kDmcMsgMotorJog = 0x0036;
constexpr uint16_t kDmcMsgMotorConfigure = 0x0037;
constexpr uint16_t kDmcMsgMotorSetSpeed = 0x0038;
constexpr uint16_t kDmcMsgMotorSetLimits = 0x0039;
constexpr uint16_t kDmcMsgMotorHardStop = 0x003A;
constexpr uint16_t kDmcMsgRtUploadBegin = 0x0100;
constexpr uint16_t kDmcMsgRtUploadAxis = 0x0101;
constexpr uint16_t kDmcMsgRtUploadDmx = 0x0102;
constexpr uint16_t kDmcMsgRtUploadEnd = 0x0103;
constexpr uint16_t kDmcMsgRtUploadTriggers = 0x0104;
constexpr uint16_t kDmcMsgRtPositionFrame = 0x0110;
constexpr uint16_t kDmcMsgRtRunMove = 0x0111;
constexpr uint16_t kDmcMsgRtGo = 0x0113;
constexpr uint16_t kDmcMsgRtEnd = 0x0114;
constexpr uint16_t kDmcMsgRtJogAll = 0x0120;

constexpr uint32_t kDmcAckOk = 0x0010;
constexpr uint32_t kDmcAckErrChecksum = 0x0011;
constexpr uint32_t kDmcAckErrMoving = 0x0012;
constexpr uint32_t kDmcAckErrUnsupported = 0x0013;
constexpr uint32_t kDmcAckErrRange = 0x0014;
constexpr uint32_t kDmcAckErrGeneral = 0x0015;

constexpr uint8_t kDmcMotorConfigEnabled = 0x01;
constexpr uint32_t kDmcCapRealTime = 0x0001;
constexpr uint32_t kDmcCapRealTimeCamera = 0x0400;
constexpr uint32_t kDmcDmxFlagFinalSet = 0x80000000U;
constexpr uint32_t kDmcGioCamShutter = 0x0001;
constexpr uint32_t kDmcGioCamMeter = 0x0002;

struct DmcFrame {
  uint32_t id = 0;
  uint16_t type = 0;
  uint16_t length = 0;
  std::vector<uint8_t> payload;
  bool valid = false;
};

uint16_t computeChecksum(const uint8_t* data, size_t bytes);
uint16_t encodeChecksum(uint16_t rawChecksum);

void appendByte(std::vector<uint8_t>& out, uint8_t value);
void appendWordLE(std::vector<uint8_t>& out, uint16_t value);
void appendDwordLE(std::vector<uint8_t>& out, uint32_t value);

uint16_t decodeWordLE(const uint8_t* ptr);
uint32_t decodeDwordLE(const uint8_t* ptr);

bool readByte(const std::vector<uint8_t>& payload, size_t offset, uint8_t* value);
bool readWordLE(const std::vector<uint8_t>& payload, size_t offset, uint16_t* value);
bool readDwordLE(const std::vector<uint8_t>& payload, size_t offset, uint32_t* value);
bool readSignedDwordLE(const std::vector<uint8_t>& payload, size_t offset, int32_t* value);

class DmcParser {
 public:
  DmcParser();
  void reset();
  bool feed(uint8_t byte, DmcFrame* frame);

 private:
  enum class State { kStart, kHeader, kData };
  State state_ = State::kStart;
  uint8_t lastByte_ = 0;
  size_t index_ = 0;
  std::array<uint8_t, kDmcMaxFrameSize> buffer_{};
};

}  // namespace sliderdmc
