#ifndef ASRS_PROTOCOL_H
#define ASRS_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#define ASRS_MAX_DATA 32
#define ASRS_DEFAULT_NODE_ID 1
#define ASRS_DEFAULT_TIMEOUT_MS 250

constexpr uint8_t ASRS_HOME_X_AXIS = 0x01;
constexpr uint8_t ASRS_HOME_Z_AXIS = 0x02;
constexpr uint8_t ASRS_HOME_VALID_MASK = ASRS_HOME_X_AXIS | ASRS_HOME_Z_AXIS;
//medium for communication:
//UART or ESPNOW
enum ASRS_Medium : uint8_t {
  ASRS_MEDIUM_UART = 0,
  ASRS_MEDIUM_ESPNOW = 1
};

//command for the asrs
enum ASRS_Command : uint8_t {
  ASRS_CMD_REQ_COORDINATES = 0x10,
  ASRS_CMD_RESP_COORDINATES = 0x11,
  ASRS_CMD_REQ_LIMITS = 0x12,
  ASRS_CMD_RESP_LIMITS = 0x13,

  ASRS_CMD_SET_MOVE = 0x20,
  ASRS_CMD_ACK = 0x21,
  ASRS_CMD_RETURN_HOME = 0x22, //homing command

  ASRS_CMD_STATUS = 0x30,
  ASRS_CMD_PAIR_REQUEST = 0x40,
  ASRS_CMD_PAIR_ACK = 0x41,
  ASRS_CMD_HEARTBEAT = 0x42,
  ASRS_CMD_HEARTBEAT_ACK = 0x43,
  ASRS_CMD_ERROR = 0x7F
};

//status of the asrs
enum ASRS_Status : uint8_t {
  ASRS_STATUS_DONE = 0,
  ASRS_STATUS_ERROR = 1,
  ASRS_STATUS_WARNING = 2,
  ASRS_STATUS_BUSY = 3,
  ASRS_STATUS_IDLE = 4
};

//error of the asrs
enum ASRS_Error : uint8_t {
  ASRS_ERROR_NONE = 0,
  ASRS_ERROR_WRONG_MODE_SELECTED = 1,
  ASRS_ERROR_INVALID_FRAME = 2,
  ASRS_ERROR_CRC_MISMATCH = 3,
  ASRS_ERROR_PAYLOAD_LENGTH = 4,
  ASRS_ERROR_ESPNOW_NOT_AVAILABLE = 5,
  ASRS_ERROR_UNKNOWN_COMMAND = 6,
  ASRS_ERROR_TIMEOUT = 7,
  ASRS_ERROR_INVALID_HOMING_REQUEST = 8,
  ASRS_ERROR_MASTER_BUSY = 9,
  ASRS_ERROR_SLAVE_BUSY = 10,
  ASRS_ERROR_LIMIT_REACHED = 11,
  ASRS_ERROR_SENSOR_MEASUREMENT_FAILED = 12
};

//data packet of the asrs communication
struct ASRS_Packet {
  uint8_t id;
  uint8_t cmd;
  uint8_t seq;
  uint8_t len;
  uint8_t data[ASRS_MAX_DATA];
};

//coordinate of the asrs (stored by the slave)
struct ASRS_Coordinates {
  int32_t x;
  int32_t z;
};

//asrs limit switches status
struct ASRS_LimitSwitches {
  bool xMinimum;
  bool xMaximum;
  bool zMinimum;
  bool zMaximum;
};

//ASRS travel command
struct ASRS_TravelCommand {
  int32_t xDistance;
  int32_t zDistance;
};

//ASRS operation status
struct ASRS_OperationStatus {
  ASRS_Status status;
  ASRS_Error error;
  uint8_t warningCode;
};

struct ASRS_HomingCommand{
  bool xHome;
  bool zHome;
};
inline void asrsClearPacket(ASRS_Packet &packet) {
  packet.id = ASRS_DEFAULT_NODE_ID;
  packet.cmd = 0;
  packet.seq = 0;
  packet.len = 0;
  memset(packet.data, 0, sizeof(packet.data));
}

inline void asrsWriteInt32(uint8_t *destination, int32_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFF);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  destination[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  destination[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline int32_t asrsReadInt32(const uint8_t *source) {
  return static_cast<int32_t>(
    static_cast<uint32_t>(source[0]) |
    (static_cast<uint32_t>(source[1]) << 8) |
    (static_cast<uint32_t>(source[2]) << 16) |
    (static_cast<uint32_t>(source[3]) << 24)
  );
}

//return the asrs communication medium.
inline const char *asrsMediumName(ASRS_Medium medium) {
  switch (medium) {
    case ASRS_MEDIUM_UART:
      return "UART";
    case ASRS_MEDIUM_ESPNOW:
      return "ESP-NOW";
    default:
      return "Unknown";
  }
}

//return the status.
inline const char *asrsStatusName(ASRS_Status status) {
  switch (status) {
    case ASRS_STATUS_DONE:
      return "Done";
    case ASRS_STATUS_ERROR: //link the error to the actual error( timeout, lost connection, wrong mode ,etc)
      return "Error";
    case ASRS_STATUS_WARNING://describe what the warning actual is
      return "Warning";
    case ASRS_STATUS_BUSY:
      return "Busy";
    case ASRS_STATUS_IDLE:
      return "Idle";
    default:
      return "Unknown";
  }
}

//returm the error name to user.
inline const char *asrsErrorName(ASRS_Error error) {
  switch (error) {
    case ASRS_ERROR_NONE:
      return "None";
    case ASRS_ERROR_WRONG_MODE_SELECTED:
      return "Wrong mode selected";
    case ASRS_ERROR_INVALID_FRAME:
      return "Invalid frame";
    case ASRS_ERROR_CRC_MISMATCH:
      return "CRC mismatch";
    case ASRS_ERROR_PAYLOAD_LENGTH:
      return "Payload length mismatch";
    case ASRS_ERROR_ESPNOW_NOT_AVAILABLE:
      return "ESP-NOW not available";
    case ASRS_ERROR_UNKNOWN_COMMAND:
      return "Unknown command";
    case ASRS_ERROR_TIMEOUT:
      return "Timeout";
    case ASRS_ERROR_INVALID_HOMING_REQUEST:
      return "Invalid homing request";
    case ASRS_ERROR_MASTER_BUSY:
      return "Master busy";
    case ASRS_ERROR_SLAVE_BUSY:
      return "Slave busy";
    case ASRS_ERROR_LIMIT_REACHED:
      return "Limit switch reached";
    case ASRS_ERROR_SENSOR_MEASUREMENT_FAILED:
      return "Sensor measurement failed";
    default:
      return "Unknown error";
  }
}

//print the complete ASRS operation status to an Arduino-compatible stream.
inline void asrsPrintOperationStatus(Print &stream, const ASRS_OperationStatus &status) {
  stream.print("Operation status: ");
  stream.print(asrsStatusName(status.status));
  if (status.status == ASRS_STATUS_ERROR) {
    stream.print(": ");
    stream.print(asrsErrorName(status.error));
  }
  stream.print(", warning code = ");
  stream.println(status.warningCode);
}

#endif
