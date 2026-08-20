#ifndef ASRS_COMM_UART_H
#define ASRS_COMM_UART_H

#include <Arduino.h>
#include "ASRS_Comm_Base.h"
#include "ASRS_FrameCodec.h"

class ASRS_Comm_UART : public ASRS_Comm_Base {
public:
  explicit ASRS_Comm_UART(HardwareSerial &serial);

  void begin(uint32_t baud, int8_t rxPin, int8_t txPin);

  bool sendPacket(ASRS_Packet &packet) override;
  bool available() override;
  bool readPacket(ASRS_Packet &packet) override;
  ASRS_Medium medium() const override;
  ASRS_Error lastError() const override;

private:
  HardwareSerial *_serial;
  ASRS_Error _lastError;
  uint8_t _sequenceCounter;
  uint8_t _rxBuffer[ASRS_FrameCodec::MAX_FRAME_SIZE];
  size_t _rxLength;

  bool extractFrame();
  ASRS_Packet _pendingPacket;
  bool _hasPendingPacket;
};

#endif
