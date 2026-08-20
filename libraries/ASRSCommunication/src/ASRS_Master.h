#ifndef ASRS_MASTER_H
#define ASRS_MASTER_H

#include "ASRS_Comm_Base.h"

class ASRS_Master {
public:
  explicit ASRS_Master(ASRS_Comm_Base &communication, uint8_t nodeId = ASRS_DEFAULT_NODE_ID);

  bool requestCoordinates(ASRS_Coordinates &coordinates, uint32_t timeoutMs = ASRS_DEFAULT_TIMEOUT_MS);
  bool requestLimitSwitches(ASRS_LimitSwitches &limits, uint32_t timeoutMs = ASRS_DEFAULT_TIMEOUT_MS);
  bool sendTravelCommand(int32_t xDistance, int32_t zDistance, uint32_t timeoutMs = ASRS_DEFAULT_TIMEOUT_MS);
  bool readOperationStatus(ASRS_OperationStatus &status);
  bool sendHomingCommand(bool xHome, bool zHome, uint32_t timeoutMs = ASRS_DEFAULT_TIMEOUT_MS);
  bool operationActive() const;
  bool coordinatesAvailable() const;
  bool lastCoordinates(ASRS_Coordinates &coordinates) const;

  ASRS_Error lastError() const;

private:
  ASRS_Comm_Base *_communication;
  uint8_t _nodeId;
  uint8_t _sequenceCounter;
  ASRS_Error _lastError;
  bool _operationActive;
  uint8_t _activeOperationSequence;
  ASRS_Command _activeOperationCommand;
  ASRS_Coordinates _coordinates;
  bool _hasCoordinates;

  ASRS_Packet makePacket(ASRS_Command command);
  bool waitForPacket(uint8_t expectedCommand,uint8_t expectedSequence, ASRS_Packet &packet, uint32_t timeoutMs);
  bool decodeCoordinates(const ASRS_Packet &packet, ASRS_Coordinates &coordinates);
  bool decodeLimits(const ASRS_Packet &packet, ASRS_LimitSwitches &limits);
  bool decodeStatus(const ASRS_Packet &packet, ASRS_OperationStatus &status);
  void clearActiveOperation();
};

#endif
