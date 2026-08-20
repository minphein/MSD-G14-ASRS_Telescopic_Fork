#ifndef ASRS_FRAME_CODEC_H
#define ASRS_FRAME_CODEC_H

#include "ASRS_Protocol.h"

namespace ASRS_FrameCodec {
  static const uint8_t START_0 = 0xA5;
  static const uint8_t START_1 = 0x5A;
  static const uint8_t VERSION = 0x01;
  static const size_t MAX_FRAME_SIZE = 2 + 5 + ASRS_MAX_DATA + 2;

  uint16_t crc16Ccitt(const uint8_t *data, size_t length);
  size_t encode(uint8_t *destination, const ASRS_Packet &packet);
  bool decode(const uint8_t *data, size_t length, ASRS_Packet &packet, ASRS_Error &error);
}

#endif
