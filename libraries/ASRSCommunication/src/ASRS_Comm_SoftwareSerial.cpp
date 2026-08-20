#include "ASRS_Comm_SoftwareSerial.h"

#if ASRS_HAS_SOFTWARESERIAL

ASRS_Comm_SoftwareSerial::ASRS_Comm_SoftwareSerial(uint8_t rxPin, uint8_t txPin)
  : _serial(rxPin, txPin),
    _lastError(ASRS_ERROR_NONE),
    _sequenceCounter(0),
    _rxLength(0),
    _hasPendingPacket(false) {
  asrsClearPacket(_pendingPacket);
}

void ASRS_Comm_SoftwareSerial::begin(uint32_t baud) {
  _serial.begin(baud);
}

bool ASRS_Comm_SoftwareSerial::sendPacket(ASRS_Packet &packet) {
  if (packet.len > ASRS_MAX_DATA) {
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  if (packet.seq == 0) {
    packet.seq = _sequenceCounter++;
  }

  uint8_t frame[ASRS_FrameCodec::MAX_FRAME_SIZE] = {};
  const size_t frameLength = ASRS_FrameCodec::encode(frame, packet);
  const bool sent = _serial.write(frame, frameLength) == frameLength;
  _lastError = sent ? ASRS_ERROR_NONE : ASRS_ERROR_TIMEOUT;
  return sent;
}

bool ASRS_Comm_SoftwareSerial::available() {
  if (_hasPendingPacket) {
    return true;
  }

  while (_serial.available() > 0 && !_hasPendingPacket) {
    if (_rxLength >= sizeof(_rxBuffer)) {
      memmove(_rxBuffer, _rxBuffer + 1, sizeof(_rxBuffer) - 1);
      _rxLength = sizeof(_rxBuffer) - 1;
    }

    _rxBuffer[_rxLength++] = static_cast<uint8_t>(_serial.read());
    extractFrame();
  }

  return _hasPendingPacket;
}

bool ASRS_Comm_SoftwareSerial::readPacket(ASRS_Packet &packet) {
  if (!available()) {
    return false;
  }

  packet = _pendingPacket;
  _hasPendingPacket = false;
  return true;
}

ASRS_Medium ASRS_Comm_SoftwareSerial::medium() const {
  return ASRS_MEDIUM_UART;
}

ASRS_Error ASRS_Comm_SoftwareSerial::lastError() const {
  return _lastError;
}

bool ASRS_Comm_SoftwareSerial::extractFrame() {
  while (_rxLength >= 2 &&
         (_rxBuffer[0] != ASRS_FrameCodec::START_0 ||
          _rxBuffer[1] != ASRS_FrameCodec::START_1)) {
    memmove(_rxBuffer, _rxBuffer + 1, _rxLength - 1);
    _rxLength--;
  }

  if (_rxLength < 7) {
    return false;
  }

  const uint8_t payloadLength = _rxBuffer[6];
  if (payloadLength > ASRS_MAX_DATA) {
    memmove(_rxBuffer, _rxBuffer + 1, _rxLength - 1);
    _rxLength--;
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  const size_t frameLength = 2 + 5 + payloadLength + 2;
  if (_rxLength < frameLength) {
    return false;
  }

  ASRS_Error error = ASRS_ERROR_NONE;
  const bool decoded = ASRS_FrameCodec::decode(_rxBuffer, frameLength, _pendingPacket, error);

  memmove(_rxBuffer, _rxBuffer + frameLength, _rxLength - frameLength);
  _rxLength -= frameLength;

  if (!decoded) {
    _lastError = error;
    return false;
  }

  _hasPendingPacket = true;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

#endif
