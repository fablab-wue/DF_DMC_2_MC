#include "dmc_protocol.h"

namespace sliderdmc {

uint16_t computeChecksum(const uint8_t* data, size_t bytes) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  size_t remaining = bytes;
  while (remaining > 0) {
    const size_t chunk = (remaining > 20) ? 20 : remaining;
    remaining -= chunk;
    for (size_t i = 0; i < chunk; ++i) {
      sum2 += sum1 += data[i];
    }
    data += chunk;
    sum1 %= 0xFF;
    sum2 %= 0xFF;
  }
  return (sum2 << 8) | sum1;
}

uint16_t encodeChecksum(uint16_t rawChecksum) {
  const uint8_t low = static_cast<uint8_t>(rawChecksum & 0xFFU);
  const uint8_t high = static_cast<uint8_t>((rawChecksum >> 8) & 0xFFU);
  const uint8_t c0 = static_cast<uint8_t>(0xFFU - ((low + high) % 0xFFU));
  const uint8_t c1 = static_cast<uint8_t>(0xFFU - ((low + c0) % 0xFFU));
  return static_cast<uint16_t>(c1 << 8) | static_cast<uint16_t>(c0);
}

void appendByte(std::vector<uint8_t>& out, uint8_t value) { out.push_back(value); }

void appendWordLE(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

void appendDwordLE(std::vector<uint8_t>& out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFU));
}

uint16_t decodeWordLE(const uint8_t* ptr) {
  return static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
}

uint32_t decodeDwordLE(const uint8_t* ptr) {
  return static_cast<uint32_t>(ptr[0]) | (static_cast<uint32_t>(ptr[1]) << 8) |
         (static_cast<uint32_t>(ptr[2]) << 16) | (static_cast<uint32_t>(ptr[3]) << 24);
}

bool readByte(const std::vector<uint8_t>& payload, size_t offset, uint8_t* value) {
  if (offset >= payload.size() || value == nullptr) {
    return false;
  }
  *value = payload[offset];
  return true;
}

bool readWordLE(const std::vector<uint8_t>& payload, size_t offset, uint16_t* value) {
  if (offset + 2 > payload.size() || value == nullptr) {
    return false;
  }
  *value = decodeWordLE(payload.data() + offset);
  return true;
}

bool readDwordLE(const std::vector<uint8_t>& payload, size_t offset, uint32_t* value) {
  if (offset + 4 > payload.size() || value == nullptr) {
    return false;
  }
  *value = decodeDwordLE(payload.data() + offset);
  return true;
}

bool readSignedDwordLE(const std::vector<uint8_t>& payload, size_t offset, int32_t* value) {
  uint32_t raw = 0;
  if (!readDwordLE(payload, offset, &raw) || value == nullptr) {
    return false;
  }
  *value = static_cast<int32_t>(raw);
  return true;
}

DmcParser::DmcParser() { reset(); }

void DmcParser::reset() {
  state_ = State::kStart;
  lastByte_ = 0;
  index_ = 0;
  buffer_.fill(0);
  buffer_[0] = 'D';
  buffer_[1] = 'F';
}

bool DmcParser::feed(uint8_t byte, DmcFrame* frame) {
  if (frame != nullptr) {
    frame->id = 0;
    frame->type = 0;
    frame->length = 0;
    frame->payload.clear();
    frame->valid = false;
  }

  switch (state_) {
    case State::kStart:
      if (byte == 'F' && lastByte_ == 'D') {
        buffer_[0] = 'D';
        buffer_[1] = 'F';
        index_ = 2;
        lastByte_ = 0;
        state_ = State::kHeader;
      } else {
        lastByte_ = byte;
      }
      return false;

    case State::kHeader:
      buffer_[index_++] = byte;
      if (index_ == kDmcHeaderSize) {
        const uint16_t length = decodeWordLE(buffer_.data() + 8);
        if (static_cast<size_t>(length) + kDmcCsumSize > kDmcMaxFrameSize - kDmcHeaderSize) {
          reset();
          return false;
        }
        state_ = State::kData;
      }
      return false;

    case State::kData:
      buffer_[index_++] = byte;
      if (index_ >= kDmcHeaderSize + static_cast<size_t>(decodeWordLE(buffer_.data() + 8)) + kDmcCsumSize) {
        state_ = State::kStart;
        if (frame == nullptr) {
          return false;
        }
        const uint16_t length = decodeWordLE(buffer_.data() + 8);
        const uint16_t type = decodeWordLE(buffer_.data() + 6);
        const uint32_t id = decodeDwordLE(buffer_.data() + 2);
        const size_t totalBytes = index_;
        const uint16_t expectedChecksum = decodeWordLE(buffer_.data() + totalBytes - kDmcCsumSize);
        const uint16_t actualChecksum = computeChecksum(buffer_.data(), totalBytes - kDmcCsumSize);
        const uint16_t encodedChecksum = encodeChecksum(actualChecksum);
        frame->id = id;
        frame->type = type;
        frame->length = length;
        frame->valid = (encodedChecksum == expectedChecksum);
        frame->payload.assign(buffer_.begin() + kDmcHeaderSize, buffer_.begin() + kDmcHeaderSize + length);
        index_ = 0;
        return true;
      }
      return false;
  }
  return false;
}

}  // namespace sliderdmc
