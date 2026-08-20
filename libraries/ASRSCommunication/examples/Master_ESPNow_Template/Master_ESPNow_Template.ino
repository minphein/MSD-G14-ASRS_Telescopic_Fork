#include <Arduino.h>
#include <ASRSCommunication.h>

#if !defined(ESP32)
#error "The ESP-NOW master template requires an ESP32-compatible board."
#endif

static constexpr uint8_t ASRS_ESPNOW_CHANNEL = 1;
static constexpr uint32_t INITIAL_PAIRING_TIMEOUT_MS = 10000;
static constexpr uint32_t WAIT_STATUS_PRINT_INTERVAL_MS = 1000;
static constexpr uint32_t BEGINNER_TEMPLATE_HEARTBEAT_INTERVAL_MS = 0;
static constexpr uint32_t BEGINNER_TEMPLATE_LINK_TIMEOUT_MS = 0;

ASRS_Comm_ESPNow espNowCommunication;
ASRS_ESPNow_MasterSession espNowSession(espNowCommunication);
ASRS_Master master(espNowCommunication);

int32_t TravelX = 200;
int32_t TravelZ = 200;

ASRS_Coordinates SlaveCoordinates;
ASRS_LimitSwitches LimitSwitches;

static void printTemplateHeader() {
  Serial.println("ASRS ESP-NOW master blocking-wrapper template.");
  Serial.println("This template is intended for robot-arm master firmware.");
  Serial.println("sendTravelCommand() and sendHomingCommand() confirm command acceptance only.");
  Serial.println("The wrapper functions wait until ASRS_STATUS_DONE before returning true.");
  Serial.println("Beginner mode uses ASRS command replies as the communication check.");
}

static bool requireConnectedSlave(const char *operationName) {
  if (espNowSession.connected()) {
    return true;
  }

  Serial.print(operationName);
  Serial.println(" rejected: ESP-NOW slave is not paired.");
  return false;
}

static bool waitForAcceptedOperationDone(const char *operationName) {
  uint32_t lastWaitingPrintMs = 0;

  while (master.operationActive()) {
    espNowSession.update();

    ASRS_OperationStatus status;
    if (!master.readOperationStatus(status)) {
      const uint32_t nowMs = millis();
      if (lastWaitingPrintMs == 0 ||
          nowMs - lastWaitingPrintMs >= WAIT_STATUS_PRINT_INTERVAL_MS) {
        lastWaitingPrintMs = nowMs;
        Serial.print("Waiting for ");
        Serial.print(operationName);
        Serial.println(" completion...");
      }
      delay(1);
      continue;
    }

    espNowSession.recordPeerActivity();
    asrsPrintOperationStatus(Serial, status);

    if (status.status == ASRS_STATUS_BUSY) {
      continue;
    }

    if (status.status == ASRS_STATUS_DONE) {
      Serial.print(operationName);
      Serial.println(" completed.");
      return true;
    }

    if (status.status == ASRS_STATUS_ERROR) {
      Serial.print(operationName);
      Serial.print(" failed: ");
      Serial.println(asrsErrorName(status.error));
      return false;
    }
  }

  Serial.print(operationName);
  Serial.println(" ended before a terminal status was observed.");
  return false;
}

bool MoveAndWait(int32_t targetX, int32_t targetZ) {
  if (!requireConnectedSlave("Move")) {
    return false;
  }

  if (!master.sendTravelCommand(targetX, targetZ)) {
    Serial.print("Move command was not accepted: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  espNowSession.recordPeerActivity();
  Serial.print("Move command accepted. Target X = ");
  Serial.print(targetX);
  Serial.print(", target Z = ");
  Serial.println(targetZ);

  if (!waitForAcceptedOperationDone("Move")) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the slave has reached targetX,targetZ.
  // Example: extend robot arm, start gripper, request a sensor, or continue the next sequence.

  TravelX += 50;
  TravelZ += 50;
  return true;
}

bool HomeAndWait(bool xHome, bool zHome) {
  if (!requireConnectedSlave("Homing")) {
    return false;
  }

  if (!master.sendHomingCommand(xHome, zHome)) {
    Serial.print("Homing command was not accepted: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  espNowSession.recordPeerActivity();
  Serial.print("Homing command accepted. X home = ");
  Serial.print(xHome ? "true" : "false");
  Serial.print(", Z home = ");
  Serial.println(zHome ? "true" : "false");

  if (!waitForAcceptedOperationDone("Homing")) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the requested homing operation is complete.
  
  return true;
}

bool ReadCoordinates(ASRS_Coordinates &coordinates) {
  if (!requireConnectedSlave("Coordinate request")) {
    return false;
  }

  if (!master.requestCoordinates(coordinates)) {
    Serial.print("Coordinate request failed: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  espNowSession.recordPeerActivity();
  Serial.print("Coordinates received: X = ");
  Serial.print(coordinates.x);
  Serial.print(", Z = ");
  Serial.println(coordinates.z);

  // USER CODE SECTION:
  // Use coordinates.x and coordinates.z here.
  
  return true;
}

bool ReadLimitSwitches(ASRS_LimitSwitches &limits) {
  if (!requireConnectedSlave("Limit-switch request")) {
    return false;
  }

  if (!master.requestLimitSwitches(limits)) {
    Serial.print("Limit-switch request failed: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  espNowSession.recordPeerActivity();
  Serial.print("Limit switches: X-min = ");
  Serial.print(limits.xMinimum ? "active" : "inactive");
  Serial.print(", X-max = ");
  Serial.print(limits.xMaximum ? "active" : "inactive");
  Serial.print(", Z-min = ");
  Serial.print(limits.zMinimum ? "active" : "inactive");
  Serial.print(", Z-max = ");
  Serial.println(limits.zMaximum ? "active" : "inactive");

  // USER CODE SECTION:
  // Use limits.xMinimum, limits.xMaximum, limits.zMinimum, and limits.zMaximum here.

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  printTemplateHeader();
  if (!espNowSession.begin(
          ASRS_ESPNOW_CHANNEL,
          INITIAL_PAIRING_TIMEOUT_MS,
          &Serial)) {
    Serial.print("ESP-NOW master session initialisation failed: ");
    Serial.println(asrsErrorName(espNowSession.lastError()));
    return;
  }

  espNowSession.setHeartbeatInterval(BEGINNER_TEMPLATE_HEARTBEAT_INTERVAL_MS);
  espNowSession.setLinkTimeout(BEGINNER_TEMPLATE_LINK_TIMEOUT_MS);

  Serial.println("Master template setup complete. Waiting for dummy slave pairing if needed.");
  Serial.println("ESP-NOW heartbeat and automatic link-timeout recovery are disabled in this beginner template.");
  Serial.println("Coordinate, limit-switch, move, and homing replies are used to verify communication.");
}

void loop() {
  espNowSession.update();
  ReadCoordinates(SlaveCoordinates);
  HomeAndWait(true,true);
  MoveAndWait(TravelX,TravelZ);
  ReadLimitSwitches(LimitSwitches);
  delay(5000);
}
