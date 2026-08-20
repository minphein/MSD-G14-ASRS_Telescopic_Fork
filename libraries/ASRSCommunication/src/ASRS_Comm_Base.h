#ifndef ASRS_COMM_BASE_H
#define ASRS_COMM_BASE_H

#include "ASRS_Protocol.h"

class ASRS_Comm_Base {
public:
  virtual ~ASRS_Comm_Base() {}

  virtual bool sendPacket(ASRS_Packet &packet) = 0;
  virtual bool available() = 0;
  virtual bool readPacket(ASRS_Packet &packet) = 0;
  virtual ASRS_Medium medium() const = 0;
  virtual ASRS_Error lastError() const = 0;
};

#endif
