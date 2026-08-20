#ifndef ASRS_COMM_SOFTWARESERIAL_H
#define ASRS_COMM_SOFTWARESERIAL_H

#include <Arduino.h>
#include "ASRS_Comm_Base.h"
#include "ASRS_FrameCodec.h"

#if __has_include(<SoftwareSerial.h>)
#include <SoftwareSerial.h>
#define ASRS_HAS_SOFTWARESERIAL 1
#else
#define ASRS_HAS_SOFTWARESERIAL 0
#endif

#if ASRS_HAS_SOFTWARESERIAL

class ASRS_Comm_SoftwareSerial : public ASRS_Comm_Base {
public:
  ASRS_Comm_SoftwareSerial(uint8_t rxPin, uint8_t txPin);

  void begin(uint32_t baud);

  bool sendPacket(ASRS_Packet &packet) override;
  bool available() override;
  bool readPacket(ASRS_Packet &packet) override;
  ASRS_Medium medium() const override;
  ASRS_Error lastError() const override;

private:
  SoftwareSerial _serial;
  ASRS_Error _lastError;
  uint8_t _sequenceCounter;
  uint8_t _rxBuffer[ASRS_FrameCodec::MAX_FRAME_SIZE];
  size_t _rxLength;

  bool extractFrame();
  ASRS_Packet _pendingPacket;
  bool _hasPendingPacket;
};

#endif

#endif
