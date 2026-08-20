#include "ASRS_Slave.h"

#if __has_include(<ASRS_Motion.h>)
#include <ASRS_Motion.h>
#define ASRS_HAS_MOTION_CONTROL 1
#else
#define ASRS_HAS_MOTION_CONTROL 0
#endif

static int32_t asrsHomeXPositionMm = 0;
static int32_t asrsHomeZPositionMm = 0;
static constexpr uint8_t ASRS_VL53L1X_TOF1_XSHUT_PIN = 10;
static constexpr uint8_t ASRS_VL53L1X_TOF2_XSHUT_PIN = 11;
static constexpr uint8_t ASRS_VL53L1X_TOF3_XSHUT_PIN = 12;
static constexpr uint8_t ASRS_VL53L1X_SDA_PIN = 8;
static constexpr uint8_t ASRS_VL53L1X_SCL_PIN = 9;
static constexpr uint8_t ASRS_VL53L1X_TOF1_ADDRESS = 0x64;
static constexpr uint8_t ASRS_VL53L1X_TOF2_ADDRESS = 0x65;
static constexpr uint8_t ASRS_VL53L1X_TOF3_ADDRESS = 0x66;
static constexpr uint8_t ASRS_VL53L1X_SENSOR_COUNT = 3;
static constexpr uint16_t ASRS_VL53L1X_INVALID_DISTANCE_MM = 65535;

static void printTofReading(uint8_t tofNumber, uint16_t distanceMm, uint8_t rangeStatus) {
  Serial.print("tof");
  Serial.print(tofNumber);
  Serial.print(": ");
  if (distanceMm == ASRS_VL53L1X_INVALID_DISTANCE_MM) {
    Serial.print("invalid");
    Serial.print(", status: ");
    Serial.println(rangeStatus);
    return;
  }

  Serial.print(distanceMm);
  Serial.print(" mm, status: ");
  Serial.println(rangeStatus);
}

ASRS_Slave::ASRS_Slave(ASRS_Comm_Base &selectedCommunication, uint8_t nodeId)
  : _selectedCommunication(&selectedCommunication),
    _wrongModeCommunication(nullptr),
    _nodeId(nodeId),
    _sequenceCounter(1),
    _lastError(ASRS_ERROR_NONE),
    _motionControlEnabled(false),
    _tofSensorsRequested(false),
    _tofSensorsReady(false),
    _coordinateRequestHandled(false),
    _hasTravelCommand(false),
    _hasHomingCommand(false),
    _operationActive(false),
    _activeOperationSequence(0),
    _activeOperationCommand(static_cast<ASRS_Command>(0)) {
      _coordinates = {0,0};
      _limits = {false, false, false, false};
      _status = {ASRS_STATUS_IDLE,
                 ASRS_ERROR_NONE,
                 0
                };
      _travelCommand = {0, 0};
      _homingCommand = {false, false};
    }

void ASRS_Slave::setWrongModeCommunication(ASRS_Comm_Base &communication) {
  _wrongModeCommunication = &communication;
}

void ASRS_Slave::update() {
  processSelectedCommunication();
  processWrongModeCommunication();
  processMotionControl();
}

void ASRS_Slave::setCoordinates(int32_t x, int32_t z) {
  _coordinates.x = x;
  _coordinates.z = z;
}

void ASRS_Slave::setLimitSwitches(bool xMinimum, bool xMaximum, bool zMinimum, bool zMaximum) {
  _limits.xMinimum = xMinimum;
  _limits.xMaximum = xMaximum;
  _limits.zMinimum = zMinimum;
  _limits.zMaximum = zMaximum;
}

void ASRS_Slave::enableMotionControl(bool enabled) {
#if ASRS_HAS_MOTION_CONTROL
  _motionControlEnabled = enabled;
  if (_motionControlEnabled) {
    asrsMotionBegin();
    refreshMotionState();
  }
#else
  _motionControlEnabled = false;
  if (enabled) {
    _lastError = ASRS_ERROR_UNKNOWN_COMMAND;
  }
#endif
}

bool ASRS_Slave::beginTofCoordinateSensors() {
  _tofSensorsRequested = true;
#if ASRS_HAS_SPARKFUN_VL53L1X
  VL53L1X_Manager::SensorConfig tofSensorConfig[] = {
      {ASRS_VL53L1X_TOF1_XSHUT_PIN, ASRS_VL53L1X_TOF1_ADDRESS},
      {ASRS_VL53L1X_TOF2_XSHUT_PIN, ASRS_VL53L1X_TOF2_ADDRESS},
      {ASRS_VL53L1X_TOF3_XSHUT_PIN, ASRS_VL53L1X_TOF3_ADDRESS}
  };

  _tofSensorsReady =
      _tofSensors.begin(
          tofSensorConfig,
          ASRS_VL53L1X_SENSOR_COUNT,
          ASRS_VL53L1X_SDA_PIN,
          ASRS_VL53L1X_SCL_PIN);
  if (!_tofSensorsReady) {
    _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
  }
  return _tofSensorsReady;
#else
  _tofSensorsReady = false;
  _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
  return false;
#endif
}

bool ASRS_Slave::tofCoordinateSensorsReady() const {
  return _tofSensorsReady;
}

bool ASRS_Slave::measureCoordinatesFromTof(bool printReadings) {
  if (!_tofSensorsReady) {
    _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
    return false;
  }

#if ASRS_HAS_SPARKFUN_VL53L1X
  uint16_t measuredDistancesMm[ASRS_VL53L1X_SENSOR_COUNT];
  uint8_t measuredRangeStatus[ASRS_VL53L1X_SENSOR_COUNT];

  for (uint8_t sensorIndex = 0; sensorIndex < ASRS_VL53L1X_SENSOR_COUNT; ++sensorIndex) {
    if (_tofSensors.updateSensorExclusiveRanging(sensorIndex)) {
      measuredDistancesMm[sensorIndex] = _tofSensors.getDistance(sensorIndex);
      measuredRangeStatus[sensorIndex] = _tofSensors.getRangeStatus(sensorIndex);
    } else {
      measuredDistancesMm[sensorIndex] = ASRS_VL53L1X_INVALID_DISTANCE_MM;
      measuredRangeStatus[sensorIndex] = _tofSensors.getRangeStatus(sensorIndex);
    }
  }

  if (printReadings) {
    for (uint8_t sensorIndex = 0; sensorIndex < ASRS_VL53L1X_SENSOR_COUNT; ++sensorIndex) {
      printTofReading(sensorIndex + 1, measuredDistancesMm[sensorIndex], measuredRangeStatus[sensorIndex]);
    }
  }

  if (measuredDistancesMm[0] == ASRS_VL53L1X_INVALID_DISTANCE_MM ||
      measuredDistancesMm[1] == ASRS_VL53L1X_INVALID_DISTANCE_MM) {
    _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
    return false;
  }

  _coordinates.x = static_cast<int32_t>(measuredDistancesMm[0]);
  _coordinates.z = static_cast<int32_t>(measuredDistancesMm[1]);
  _lastError = ASRS_ERROR_NONE;
  return true;
#else
  _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
  return false;
#endif
}

void ASRS_Slave::setHomePosition(int32_t xHomeMm, int32_t zHomeMm) {
  asrsHomeXPositionMm = xHomeMm;
  asrsHomeZPositionMm = zHomeMm;
}

//function to set the operation status of the slave
void ASRS_Slave::setOperationStatus(ASRS_Status status, ASRS_Error error, uint8_t warningCode) {
  _status.status = status;
  _status.error = error;
  _status.warningCode = warningCode;
}

bool ASRS_Slave::publishOperationStatus() {
  return sendStatus(*_selectedCommunication, 0);
}

bool ASRS_Slave::travelCommandAvailable() const {
  return _hasTravelCommand;
}

bool ASRS_Slave::readTravelCommand(ASRS_TravelCommand &command) {
  if (!_hasTravelCommand) {
    return false;
  }

  command = _travelCommand;
  _hasTravelCommand = false;
  return true;
}

bool ASRS_Slave::coordinateRequestHandled() {
  if (!_coordinateRequestHandled) {
    return false;
  }

  _coordinateRequestHandled = false;
  return true;
}

bool ASRS_Slave::homingCommandAvailable() const{
  return _hasHomingCommand;
}

bool ASRS_Slave::readHomingCommand(ASRS_HomingCommand &command){
  if(!_hasHomingCommand){
    return false;
  }

  command = _homingCommand;
  //set the hashomingcommand attribute to false for future command.
  _hasHomingCommand = false;
  return true;
}

bool ASRS_Slave::operationActive() const{
  return _operationActive;
}
//this function is to report execution has started (operation set to busy)
bool ASRS_Slave::reportOperationBusy(){
  if(!_operationActive){
    return false;
  }
  //set the status to busy, no error and warning code =0
  setOperationStatus(ASRS_STATUS_BUSY,ASRS_ERROR_NONE,0);
  //send the status in packet to the master.
  return sendStatus(*_selectedCommunication,_activeOperationSequence);
}

//Report successful completion to the master.
bool ASRS_Slave::completeActiveOperation(){
  if(!_operationActive){
    return false;
  }

  setOperationStatus(ASRS_STATUS_DONE, ASRS_ERROR_NONE,0);

  const bool sent = sendStatus(*_selectedCommunication, _activeOperationSequence);
  if(sent){
    clearActiveOperation();
  }
  return sent;
}

//report failure
bool ASRS_Slave::failActiveOperation(ASRS_Error error){
  if(!_operationActive){
    return false;
  }

  setOperationStatus(ASRS_STATUS_ERROR, error, 0);

  const bool sent = sendStatus(*_selectedCommunication, _activeOperationSequence);

  if(sent){
    clearActiveOperation();
  }

  return sent;
}


ASRS_Error ASRS_Slave::lastError() const {
  return _lastError;
}

void ASRS_Slave::processSelectedCommunication() {
  while (_selectedCommunication->available()) {
    ASRS_Packet packet;
    if (_selectedCommunication->readPacket(packet)) {
      processPacket(*_selectedCommunication, packet, false);
    } else {
      _lastError = _selectedCommunication->lastError();
    }
  }
}

void ASRS_Slave::processWrongModeCommunication() {
  if (_wrongModeCommunication == nullptr || _wrongModeCommunication == _selectedCommunication) {
    return;
  }

  while (_wrongModeCommunication->available()) {
    ASRS_Packet packet;
    if (_wrongModeCommunication->readPacket(packet)) {
      processPacket(*_wrongModeCommunication, packet, true);
    } else {
      _lastError = _wrongModeCommunication->lastError();
    }
  }
}

void ASRS_Slave::processMotionControl() {
#if ASRS_HAS_MOTION_CONTROL
  if (!_motionControlEnabled) {
    return;
  }

  asrsMotionUpdate();
  refreshMotionState();

  if (!_operationActive) {
    return;
  }

  if (asrsMotionFinished() && asrsMotionLimitFault()) {
    if (!failActiveOperation(ASRS_ERROR_LIMIT_REACHED)) {
      clearActiveOperation();
    }
    asrsMotionClearFinished();
    return;
  }

  if (asrsMotionFinished()) {
    const ASRS_Command completedCommand = _activeOperationCommand;
    if (completeActiveOperation()) {
      if (completedCommand == ASRS_CMD_RETURN_HOME) {
        Serial.println("done home");
      } else if (completedCommand == ASRS_CMD_SET_MOVE) {
        Serial.println("done moved");
      }
      asrsMotionClearFinished();
    }
  }
#endif
}

bool ASRS_Slave::startAcceptedMotion(
    int32_t xDistanceMm,
    int32_t zDistanceMm,
    ASRS_Error startError) {
#if ASRS_HAS_MOTION_CONTROL
  if (!asrsMotionStartMove(xDistanceMm, zDistanceMm)) {
    failActiveOperation(startError);
    return false;
  }

  refreshMotionState();
  return reportOperationBusy();
#else
  (void)xDistanceMm;
  (void)zDistanceMm;
  failActiveOperation(startError);
  return false;
#endif
}

bool ASRS_Slave::startAcceptedXMinimumHoming(ASRS_Error startError) {
#if ASRS_HAS_MOTION_CONTROL
  if (!asrsMotionStartHomeXToMinimum()) {
    failActiveOperation(startError);
    return false;
  }

  refreshMotionState();
  return reportOperationBusy();
#else
  failActiveOperation(startError);
  return false;
#endif
}

bool ASRS_Slave::startAcceptedZMinimumHoming(ASRS_Error startError) {
#if ASRS_HAS_MOTION_CONTROL
  if (!asrsMotionStartHomeZToMinimum()) {
    failActiveOperation(startError);
    return false;
  }

  refreshMotionState();
  return reportOperationBusy();
#else
  failActiveOperation(startError);
  return false;
#endif
}

bool ASRS_Slave::startAcceptedZThenXHoming(ASRS_Error startError) {
#if ASRS_HAS_MOTION_CONTROL
  if (!asrsMotionStartHomeZThenXToMinimum()) {
    failActiveOperation(startError);
    return false;
  }

  refreshMotionState();
  return reportOperationBusy();
#else
  failActiveOperation(startError);
  return false;
#endif
}

void ASRS_Slave::refreshMotionState() {
#if ASRS_HAS_MOTION_CONTROL
  if (!_tofSensorsReady) {
    _coordinates.x = asrsMotionEstimatedXMillimetres();
    _coordinates.z = asrsMotionEstimatedZMillimetres();
  }
  _limits.xMinimum = asrsMotionXMinLimitActive();
  _limits.xMaximum = asrsMotionXMaxLimitActive();
  _limits.zMinimum = asrsMotionZMinLimitActive();
  _limits.zMaximum = asrsMotionZMaxLimitActive();
#endif
}

//function to process the packet received from master.
void ASRS_Slave::processPacket(ASRS_Comm_Base &communication, ASRS_Packet &packet, bool wrongMode) {
  if (wrongMode) {
    sendError(communication, ASRS_ERROR_WRONG_MODE_SELECTED, packet.seq);
    _lastError = ASRS_ERROR_WRONG_MODE_SELECTED;
    return;
  }

  switch (packet.cmd) {
    case ASRS_CMD_REQ_COORDINATES:
      if (_tofSensorsRequested) {
        if (_operationActive) {
          sendError(communication, ASRS_ERROR_SLAVE_BUSY, packet.seq);
          _lastError = ASRS_ERROR_SLAVE_BUSY;
          return;
        }
        if (!measureCoordinatesFromTof(true)) {
          sendError(communication, ASRS_ERROR_SENSOR_MEASUREMENT_FAILED, packet.seq);
          _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
          return;
        }
      }
      if (sendCoordinates(communication, packet.seq)) {
        _coordinateRequestHandled = true;
      }
      break;

    case ASRS_CMD_REQ_LIMITS:
      sendLimitSwitches(communication, packet.seq);
      break;
    //handle the moving command
    case ASRS_CMD_SET_MOVE:{
      //check is pakcet length is not enough, send error to master
      if(packet.len != 8){
        sendError(communication, ASRS_ERROR_PAYLOAD_LENGTH, packet.seq);
        _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
        return;
      }
      if(_operationActive){
        sendError(communication,ASRS_ERROR_SLAVE_BUSY, packet.seq);
        _lastError = ASRS_ERROR_SLAVE_BUSY;
        return;
      }
      const int32_t targetX = asrsReadInt32(&packet.data[0]);
      const int32_t targetZ = asrsReadInt32(&packet.data[4]);
      if (_tofSensorsRequested && !measureCoordinatesFromTof(true)) {
        sendError(communication, ASRS_ERROR_SENSOR_MEASUREMENT_FAILED, packet.seq);
        _lastError = ASRS_ERROR_SENSOR_MEASUREMENT_FAILED;
        return;
      }
      const int32_t xMoveMm = targetX - _coordinates.x;
      const int32_t zMoveMm = targetZ - _coordinates.z;

      _travelCommand.xDistance = targetX;
      _travelCommand.zDistance = targetZ;

      _activeOperationCommand = ASRS_CMD_SET_MOVE;
      _activeOperationSequence = packet.seq;

      _operationActive = true;
      _hasTravelCommand = true;
      _hasHomingCommand = false;

      if (sendAcknowledgement(communication,_activeOperationSequence)) {
        _lastError = ASRS_ERROR_NONE;
        if (_motionControlEnabled) {
          startAcceptedMotion(
              xMoveMm,
              zMoveMm,
              ASRS_ERROR_SLAVE_BUSY);
        }
      } else {
        clearActiveOperation();
      }
      break;
    }
    //handle the homing command.
    case ASRS_CMD_RETURN_HOME:{
      if(packet.len != 1){
        sendError(communication, ASRS_ERROR_PAYLOAD_LENGTH,packet.seq);
        _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
        return;
      }

      const uint8_t homeMask = packet.data[0];

      if(homeMask == 0 || (homeMask &~ASRS_HOME_VALID_MASK) != 0){
        sendError(communication, ASRS_ERROR_INVALID_HOMING_REQUEST,packet.seq);
        _lastError = ASRS_ERROR_INVALID_HOMING_REQUEST;
        return;
      }

      if(_operationActive){
        sendError(communication, ASRS_ERROR_SLAVE_BUSY,packet.seq);
        _lastError = ASRS_ERROR_SLAVE_BUSY;
        return;
      }
      _homingCommand.xHome = (homeMask & ASRS_HOME_X_AXIS) != 0;
      _homingCommand.zHome = (homeMask & ASRS_HOME_Z_AXIS) != 0;

      _activeOperationCommand = ASRS_CMD_RETURN_HOME;
      _activeOperationSequence = packet.seq;

      _operationActive = true;
      _hasTravelCommand = false;
      _hasHomingCommand = true;

      if (sendAcknowledgement(communication,_activeOperationSequence)) {
        _lastError = ASRS_ERROR_NONE;
        if (_motionControlEnabled) {
          if (_homingCommand.xHome && _homingCommand.zHome) {
            startAcceptedZThenXHoming(ASRS_ERROR_INVALID_HOMING_REQUEST);
          } else if (_homingCommand.zHome) {
            startAcceptedZMinimumHoming(ASRS_ERROR_INVALID_HOMING_REQUEST);
          } else {
            startAcceptedXMinimumHoming(ASRS_ERROR_INVALID_HOMING_REQUEST);
          }
        }
      } else {
        clearActiveOperation();
      }
      break;
    }

    default:
      sendError(communication, ASRS_ERROR_UNKNOWN_COMMAND, packet.seq);
      _lastError = ASRS_ERROR_UNKNOWN_COMMAND;
      break;
  }
}

bool ASRS_Slave::sendCoordinates(ASRS_Comm_Base &communication, uint8_t sequence) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = ASRS_CMD_RESP_COORDINATES;
  packet.seq = sequence;
  packet.len = 8;
  asrsWriteInt32(&packet.data[0], _coordinates.x);
  asrsWriteInt32(&packet.data[4], _coordinates.z);
  return communication.sendPacket(packet);
}

bool ASRS_Slave::sendLimitSwitches(ASRS_Comm_Base &communication, uint8_t sequence) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = ASRS_CMD_RESP_LIMITS;
  packet.seq = sequence;
  packet.len = 4;
  packet.data[0] = static_cast<uint8_t>(_limits.xMinimum);
  packet.data[1] = static_cast<uint8_t>(_limits.xMaximum);
  packet.data[2] = static_cast<uint8_t>(_limits.zMinimum);
  packet.data[3] = static_cast<uint8_t>(_limits.zMaximum);
  return communication.sendPacket(packet);
}

bool ASRS_Slave::sendAcknowledgement(ASRS_Comm_Base &communication, uint8_t sequence) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = ASRS_CMD_ACK;
  packet.seq = sequence;
  return communication.sendPacket(packet);
}

bool ASRS_Slave::sendStatus(ASRS_Comm_Base &communication, uint8_t sequence) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = ASRS_CMD_STATUS;
  packet.seq = sequence == 0 ? _sequenceCounter++ : sequence;
  packet.len = 3;
  packet.data[0] = static_cast<uint8_t>(_status.status);
  packet.data[1] = static_cast<uint8_t>(_status.error);
  packet.data[2] = _status.warningCode;
  return communication.sendPacket(packet);
}

bool ASRS_Slave::sendError(ASRS_Comm_Base &communication, ASRS_Error error, uint8_t sequence) {
  ASRS_Packet packet;
  asrsClearPacket(packet);
  packet.id = _nodeId;
  packet.cmd = ASRS_CMD_ERROR;
  packet.seq = sequence;
  packet.len = 1;
  packet.data[0] = static_cast<uint8_t>(error);
  return communication.sendPacket(packet);
}

//clear the completed operation
void ASRS_Slave::clearActiveOperation(){
  _operationActive = false;
  _activeOperationSequence = 0;
  _activeOperationCommand = static_cast<ASRS_Command>(0); // same as giving 0x00

  _hasTravelCommand = false;
  _hasHomingCommand = false;

  _travelCommand = {0,0};
  _homingCommand = {false, false};

  setOperationStatus(ASRS_STATUS_IDLE, ASRS_ERROR_NONE,0);
}
