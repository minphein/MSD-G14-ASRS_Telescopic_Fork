#include "ASRS_FrameCodec.h"

uint16_t ASRS_FrameCodec::crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

size_t ASRS_FrameCodec::encode(uint8_t *destination, const ASRS_Packet &packet) {
  destination[0] = START_0;
  destination[1] = START_1;
  destination[2] = VERSION;
  destination[3] = packet.id;
  destination[4] = packet.cmd;
  destination[5] = packet.seq;
  destination[6] = packet.len;

  if (packet.len > 0) {
    memcpy(&destination[7], packet.data, packet.len);
  }

  const uint16_t crc = crc16Ccitt(&destination[2], 5 + packet.len);
  destination[7 + packet.len] = static_cast<uint8_t>(crc & 0xFF);
  destination[8 + packet.len] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  return 2 + 5 + packet.len + 2;
}

bool ASRS_FrameCodec::decode(const uint8_t *data, size_t length, ASRS_Packet &packet, ASRS_Error &error) {
  error = ASRS_ERROR_NONE;

  if (length < 9 || data[0] != START_0 || data[1] != START_1 || data[2] != VERSION) {
    error = ASRS_ERROR_INVALID_FRAME;
    return false;
  }

  const uint8_t payloadLength = data[6];
  const size_t expectedLength = 2 + 5 + payloadLength + 2;

  if (payloadLength > ASRS_MAX_DATA || length != expectedLength) {
    error = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(data[7 + payloadLength]) |
                               (static_cast<uint16_t>(data[8 + payloadLength]) << 8);
  const uint16_t calculatedCrc = crc16Ccitt(&data[2], 5 + payloadLength);

  if (receivedCrc != calculatedCrc) {
    error = ASRS_ERROR_CRC_MISMATCH;
    return false;
  }

  packet.id = data[3];
  packet.cmd = data[4];
  packet.seq = data[5];
  packet.len = payloadLength;
  memset(packet.data, 0, sizeof(packet.data));

  if (payloadLength > 0) {
    memcpy(packet.data, &data[7], payloadLength);
  }

  return true;
}
