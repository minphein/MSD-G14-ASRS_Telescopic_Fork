#ifndef ASRS_RACK_PROTOCOL_H
#define ASRS_RACK_PROTOCOL_H

#include <Arduino.h>
#include <string.h>

constexpr uint32_t ASRS_RACK_MAGIC = 0x46524B31UL; // "FRK1"
constexpr uint8_t ASRS_RACK_VERSION = 1;
constexpr uint8_t ASRS_RACK_STATUS = 1;
constexpr uint8_t ASRS_RACK_STATUS_REQUEST = 2;

struct __attribute__((packed)) ASRS_RackPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t sequence;
  uint8_t slots[4];
  uint32_t uptimeMs;
};

inline bool asrsIsRackPacket(const uint8_t *data, int length) {
  if (data == nullptr || length != static_cast<int>(sizeof(ASRS_RackPacket))) {
    return false;
  }
  ASRS_RackPacket packet;
  memcpy(&packet, data, sizeof(packet));
  return packet.magic == ASRS_RACK_MAGIC &&
         packet.version == ASRS_RACK_VERSION;
}

#endif
