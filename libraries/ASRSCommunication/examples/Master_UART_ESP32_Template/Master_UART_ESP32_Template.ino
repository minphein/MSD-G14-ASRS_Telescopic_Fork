#include <Arduino.h>
#include <ASRSCommunication.h>

static constexpr int8_t ASRS_UART_RX_PIN = 18;
static constexpr int8_t ASRS_UART_TX_PIN = 17;
static constexpr uint32_t ASRS_UART_BAUD = 9600;
static constexpr uint32_t MOVE_WAIT_TIMEOUT_MS = 30000;
static constexpr uint32_t HOMING_WAIT_TIMEOUT_MS = 0;
static constexpr uint32_t WAIT_STATUS_PRINT_INTERVAL_MS = 1000;

ASRS_Comm_UART asrsCommunication(Serial1);
ASRS_Master master(asrsCommunication);

ASRS_Coordinates SlaveCoordinates;
ASRS_LimitSwitches LimitSwitches;

int32_t TravelX = 200;
int32_t TravelZ = 200;

static void printTemplateHeader() {
  Serial.println("ASRS ESP32 UART master template.");
  Serial.println("USB Serial remains available for debug output.");
  Serial.print("Default ASRS UART pins: GPIO");
  Serial.print(ASRS_UART_RX_PIN);
  Serial.print(" = RX, GPIO");
  Serial.print(ASRS_UART_TX_PIN);
  Serial.println(" = TX.");
  Serial.println("Connect master TX to slave RX, master RX to slave TX, and connect GND.");
  Serial.println("When used with Template_Slave, set the slave GPIO2 LOW and reset the slave.");
}

static bool waitForAcceptedOperationDone(const char *operationName,
                                         uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  uint32_t lastWaitingPrintMs = 0;

  while (master.operationActive()) {
    ASRS_OperationStatus status;
    if (!master.readOperationStatus(status)) {
      const uint32_t nowMs = millis();
      if (timeoutMs != 0 && nowMs - startMs > timeoutMs) {
        Serial.print(operationName);
        Serial.println(" timed out while waiting for DONE or ERROR.");
        return false;
      }

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
  if (!master.sendTravelCommand(targetX, targetZ)) {
    Serial.print("Move command was not accepted: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print("Move command accepted. Target X = ");
  Serial.print(targetX);
  Serial.print(", target Z = ");
  Serial.println(targetZ);

  if (!waitForAcceptedOperationDone("Move", MOVE_WAIT_TIMEOUT_MS)) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the slave has reached targetX,targetZ.

  return true;
}

bool HomeAndWait(bool xHome, bool zHome) {
  if (!master.sendHomingCommand(xHome, zHome)) {
    Serial.print("Homing command was not accepted: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print("Homing command accepted. X home = ");
  Serial.print(xHome ? "true" : "false");
  Serial.print(", Z home = ");
  Serial.println(zHome ? "true" : "false");

  if (!waitForAcceptedOperationDone("Homing", HOMING_WAIT_TIMEOUT_MS)) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the requested homing operation is complete.

  return true;
}

bool ReadCoordinates(ASRS_Coordinates &coordinates) {
  if (!master.requestCoordinates(coordinates)) {
    Serial.print("Coordinate request failed: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print("Coordinates received: X = ");
  Serial.print(coordinates.x);
  Serial.print(", Z = ");
  Serial.println(coordinates.z);

  // USER CODE SECTION:
  // Use coordinates.x and coordinates.z here.

  return true;
}

bool ReadLimitSwitches(ASRS_LimitSwitches &limits) {
  if (!master.requestLimitSwitches(limits)) {
    Serial.print("Limit-switch request failed: ");
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

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
  asrsCommunication.begin(ASRS_UART_BAUD, ASRS_UART_RX_PIN, ASRS_UART_TX_PIN);

  Serial.println("ASRS hardware UART link started.");
}

void loop() {
  ReadCoordinates(SlaveCoordinates);
  HomeAndWait(true,true);
  MoveAndWait(TravelX,TravelZ);
  ReadLimitSwitches(LimitSwitches);
  delay(5000);
}
