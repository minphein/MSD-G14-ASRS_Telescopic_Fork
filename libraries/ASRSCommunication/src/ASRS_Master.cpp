#include "ASRS_Master.h"

ASRS_Master::ASRS_Master(ASRS_Comm_Base &communication, uint8_t nodeId)
  : _communication(&communication),
    _nodeId(nodeId),
    _sequenceCounter(1),
    _lastError(ASRS_ERROR_NONE),
    _operationActive(false),
    _activeOperationSequence(0),
    _activeOperationCommand(static_cast<ASRS_Command>(0)),
    _coordinates{0, 0},
    _hasCoordinates(false) {
}

bool ASRS_Master::requestCoordinates(ASRS_Coordinates &coordinates, uint32_t timeoutMs) {
  if (_operationActive) {
    _lastError = ASRS_ERROR_MASTER_BUSY;
    return false;
  }

  //make a packet of request coordinate.
  ASRS_Packet request = makePacket(ASRS_CMD_REQ_COORDINATES);
  //if sendPacket success, would skip below block of code.
  if (!_communication->sendPacket(request)) {
    _lastError = _communication->lastError();
    return false;
  }
  //create a response packet.
  ASRS_Packet response;
  if (!waitForPacket(ASRS_CMD_RESP_COORDINATES,request.seq, response, timeoutMs)) {
    return false;
  }

  return decodeCoordinates(response, coordinates);
}

bool ASRS_Master::requestLimitSwitches(ASRS_LimitSwitches &limits, uint32_t timeoutMs) {
  if (_operationActive) {
    _lastError = ASRS_ERROR_MASTER_BUSY;
    return false;
  }

  ASRS_Packet request = makePacket(ASRS_CMD_REQ_LIMITS);

  if (!_communication->sendPacket(request)) {
    _lastError = _communication->lastError();
    return false;
  }

  ASRS_Packet response;
  if (!waitForPacket(ASRS_CMD_RESP_LIMITS, request.seq,response, timeoutMs)) {
    return false;
  }

  return decodeLimits(response, limits);
}

bool ASRS_Master::sendTravelCommand(int32_t xDistance, int32_t zDistance, uint32_t timeoutMs) {
  if (_operationActive) {
    _lastError = ASRS_ERROR_MASTER_BUSY;
    return false;
  }

  ASRS_Packet command = makePacket(ASRS_CMD_SET_MOVE);
  command.len = 8;
  asrsWriteInt32(&command.data[0], xDistance);
  asrsWriteInt32(&command.data[4], zDistance);

  if (!_communication->sendPacket(command)) {
    _lastError = _communication->lastError();
    return false;
  }

  ASRS_Packet acknowledgement;
  if (!waitForPacket(ASRS_CMD_ACK, command.seq, acknowledgement, timeoutMs)) {
    return false;
  }

  _operationActive = true;
  _activeOperationSequence = command.seq;
  _activeOperationCommand = ASRS_CMD_SET_MOVE;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Master::readOperationStatus(ASRS_OperationStatus &status) {
  if (!_operationActive || !_communication->available()) {
    return false;
  }

  ASRS_Packet packet;
  if (!_communication->readPacket(packet)) {
    _lastError = _communication->lastError();
    return false;
  }

  if (packet.id != _nodeId ||
      packet.seq != _activeOperationSequence) {
    return false;
  }

  if (packet.cmd != ASRS_CMD_STATUS &&
      packet.cmd != ASRS_CMD_ERROR) {
    _lastError = ASRS_ERROR_UNKNOWN_COMMAND;
    return false;
  }

  if (!decodeStatus(packet, status)) {
    return false;
  }

  if (status.status == ASRS_STATUS_DONE ||
      status.status == ASRS_STATUS_ERROR) {
    clearActiveOperation();
  }

  return true;
}

bool ASRS_Master::sendHomingCommand(bool xHome, bool zHome, uint32_t timeoutMs){
  if (_operationActive) {
    _lastError = ASRS_ERROR_MASTER_BUSY;
    return false;
  }

  uint8_t homeMask = 0;
  if(xHome){
    homeMask |= ASRS_HOME_X_AXIS;
  }

  if(zHome){
    homeMask |= ASRS_HOME_Z_AXIS;
  }

  if(homeMask == 0){
    _lastError = ASRS_ERROR_INVALID_HOMING_REQUEST;
    return false;
  }

  ASRS_Packet command = makePacket(ASRS_CMD_RETURN_HOME);
  command.len = 1;
  command.data[0] = homeMask;

  if(!_communication->sendPacket(command)){
    _lastError = _communication->lastError();
    return false;
  }

  ASRS_Packet acknowledgement;
  if (!waitForPacket(ASRS_CMD_ACK, command.seq, acknowledgement, timeoutMs)) {
    return false;
  }

  _operationActive = true;
  _activeOperationSequence = command.seq;
  _activeOperationCommand = ASRS_CMD_RETURN_HOME;
  _lastError = ASRS_ERROR_NONE;
  return true;

}

bool ASRS_Master::operationActive() const{
  return _operationActive;
}

bool ASRS_Master::coordinatesAvailable() const {
  return _hasCoordinates;
}

bool ASRS_Master::lastCoordinates(ASRS_Coordinates &coordinates) const {
  if (!_hasCoordinates) {
    return false;
  }

  coordinates = _coordinates;
  return true;
}

ASRS_Error ASRS_Master::lastError() const {
  return _lastError;
}

ASRS_Packet ASRS_Master::makePacket(ASRS_Command command) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = static_cast<uint8_t>(command);
  packet.seq = _sequenceCounter++;
  return packet;
}

bool ASRS_Master::waitForPacket(uint8_t expectedCommand,uint8_t expectedSequence,ASRS_Packet &packet,uint32_t timeoutMs) {

  const uint32_t startTime = millis();

  while (millis() - startTime < timeoutMs) {
    if (!_communication->available()) {
      delay(1);
      continue;
    }

    if (!_communication->readPacket(packet)) {
      _lastError = _communication->lastError();
      return false;
    }

    if (packet.id != _nodeId ||
        packet.seq != expectedSequence) {
      continue;
    }

    if (packet.cmd == ASRS_CMD_ERROR) {
      ASRS_OperationStatus status;

      if (!decodeStatus(packet, status)) {
        return false;
      }

      _lastError = status.error;
      return false;
    }

    if (packet.cmd == expectedCommand) {
      _lastError = ASRS_ERROR_NONE;
      return true;
    }
  }

  _lastError = ASRS_ERROR_TIMEOUT;
  return false;
}

void ASRS_Master::clearActiveOperation() {
  _operationActive = false;
  _activeOperationSequence = 0;
  _activeOperationCommand = static_cast<ASRS_Command>(0);
}

bool ASRS_Master::decodeCoordinates(const ASRS_Packet &packet, ASRS_Coordinates &coordinates) {
  if (packet.len != 8) {
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  coordinates.x = asrsReadInt32(&packet.data[0]);
  coordinates.z = asrsReadInt32(&packet.data[4]);
  _coordinates = coordinates;
  _hasCoordinates = true;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Master::decodeLimits(const ASRS_Packet &packet, ASRS_LimitSwitches &limits) {
  if (packet.len != 4) {
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  limits.xMinimum = packet.data[0] != 0;
  limits.xMaximum = packet.data[1] != 0;
  limits.zMinimum = packet.data[2] != 0;
  limits.zMaximum = packet.data[3] != 0;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Master::decodeStatus(const ASRS_Packet &packet, ASRS_OperationStatus &status) {
  if (packet.cmd == ASRS_CMD_ERROR) {
    if (packet.len != 1) {
      _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
      return false;
    }

    status.status = ASRS_STATUS_ERROR;
    status.error = static_cast<ASRS_Error>(packet.data[0]);
    status.warningCode = 0;
    _lastError = status.error;
    return true;
  }

  if (packet.len != 3) {
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  status.status = static_cast<ASRS_Status>(packet.data[0]);
  status.error = static_cast<ASRS_Error>(packet.data[1]);
  status.warningCode = packet.data[2];
  _lastError = status.error;
  return true;
}
