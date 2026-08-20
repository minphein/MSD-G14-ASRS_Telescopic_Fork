#ifndef ASRS_SLAVE_H
#define ASRS_SLAVE_H

#include "ASRS_Comm_Base.h"
#include "VL53L1X_Manager.h"

class ASRS_Slave {
public:
  ASRS_Slave(ASRS_Comm_Base &selectedCommunication, uint8_t nodeId = ASRS_DEFAULT_NODE_ID);

  void setWrongModeCommunication(ASRS_Comm_Base &communication);
  void update();

  void setCoordinates(int32_t x, int32_t z);
  void setLimitSwitches(bool xMinimum, bool xMaximum, bool zMinimum, bool zMaximum);
  void enableMotionControl(bool enabled = true);
  bool beginTofCoordinateSensors();
  bool tofCoordinateSensorsReady() const;
  bool measureCoordinatesFromTof(bool printReadings = true);
  void setHomePosition(int32_t xHomeMm, int32_t zHomeMm);
  void setOperationStatus(ASRS_Status status, ASRS_Error error = ASRS_ERROR_NONE, uint8_t warningCode = 0);
  bool publishOperationStatus();

  bool travelCommandAvailable() const;
  bool readTravelCommand(ASRS_TravelCommand &command);
  bool coordinateRequestHandled();
  bool homingCommandAvailable() const;
  bool readHomingCommand(ASRS_HomingCommand &command);

  bool operationActive() const;

  bool reportOperationBusy();
  bool completeActiveOperation();
  bool failActiveOperation(ASRS_Error error);
  ASRS_Error lastError() const;

private:
  ASRS_Comm_Base *_selectedCommunication;
  ASRS_Comm_Base *_wrongModeCommunication;
  uint8_t _nodeId;
  uint8_t _sequenceCounter;
  ASRS_Error _lastError;

  ASRS_Coordinates _coordinates;
  ASRS_LimitSwitches _limits;
  ASRS_OperationStatus _status;
  bool _motionControlEnabled;
  bool _tofSensorsRequested;
  bool _tofSensorsReady;
#if ASRS_HAS_SPARKFUN_VL53L1X
  VL53L1X_Manager _tofSensors;
#endif
  bool _coordinateRequestHandled;
  //for handling travel command from master
  ASRS_TravelCommand _travelCommand;
  bool _hasTravelCommand;
  //for handling homing command from master
  ASRS_HomingCommand _homingCommand;
  bool _hasHomingCommand;
  //for make sure the slave is handling the known sequence of command.
  bool _operationActive;
  uint8_t _activeOperationSequence;
  ASRS_Command _activeOperationCommand;

  void processSelectedCommunication();
  void processWrongModeCommunication();
  void processMotionControl();
  void processPacket(ASRS_Comm_Base &communication, ASRS_Packet &packet, bool wrongMode);
  bool startAcceptedMotion(int32_t xDistanceMm, int32_t zDistanceMm, ASRS_Error startError);
  bool startAcceptedXMinimumHoming(ASRS_Error startError);
  bool startAcceptedZMinimumHoming(ASRS_Error startError);
  bool startAcceptedZThenXHoming(ASRS_Error startError);
  void refreshMotionState();

  bool sendCoordinates(ASRS_Comm_Base &communication, uint8_t sequence);
  bool sendLimitSwitches(ASRS_Comm_Base &communication, uint8_t sequence);
  bool sendAcknowledgement(ASRS_Comm_Base &communication, uint8_t sequence);
  bool sendStatus(ASRS_Comm_Base &communication, uint8_t sequence);
  bool sendError(ASRS_Comm_Base &communication, ASRS_Error error, uint8_t sequence);
  void clearActiveOperation();
};

#endif
