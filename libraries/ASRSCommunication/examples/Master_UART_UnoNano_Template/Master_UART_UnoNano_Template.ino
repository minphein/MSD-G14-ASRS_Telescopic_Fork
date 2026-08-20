#include <Arduino.h>
#include <ASRSCommunication.h>

#if !ASRS_HAS_SOFTWARESERIAL
#error "This template requires Arduino SoftwareSerial support."
#endif

static constexpr uint8_t ASRS_SOFTSERIAL_RX_PIN = 10;
static constexpr uint8_t ASRS_SOFTSERIAL_TX_PIN = 11;

static constexpr uint32_t ASRS_SOFTSERIAL_BAUD = 9600;
static constexpr uint32_t MOVE_WAIT_TIMEOUT_MS = 30000;
static constexpr uint32_t HOMING_WAIT_TIMEOUT_MS = 0;
static constexpr uint32_t WAIT_STATUS_PRINT_INTERVAL_MS = 1000;

ASRS_LimitSwitches SlaveLimitSwitches;
ASRS_Coordinates SlaveCoordinates;

int32_t TravelX = 200;
int32_t TravelZ = 200;

ASRS_Comm_SoftwareSerial asrsCommunication(
    ASRS_SOFTSERIAL_RX_PIN,
    ASRS_SOFTSERIAL_TX_PIN);
ASRS_Master master(asrsCommunication);

static void printTemplateHeader() {
  Serial.println(F("ASRS Uno/Nano SoftwareSerial master template."));
  Serial.println(F("USB Serial remains available for debug output."));
  Serial.println(F("Default ASRS SoftwareSerial pins: D10 = RX, D11 = TX."));
  Serial.println(F("D10/D11 are configurable. Avoid D0/D1 when Serial Monitor debugging is required."));
  Serial.println(F("Use level shifting when a 5 V Arduino TX pin connects to an ESP32-S3 RX pin."));
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
        Serial.println(F(" timed out while waiting for DONE or ERROR."));
        return false;
      }

      if (lastWaitingPrintMs == 0 ||
          nowMs - lastWaitingPrintMs >= WAIT_STATUS_PRINT_INTERVAL_MS) {
        lastWaitingPrintMs = nowMs;
        Serial.print(F("Waiting for "));
        Serial.print(operationName);
        Serial.println(F(" completion..."));
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
      Serial.println(F(" completed."));
      return true;
    }

    if (status.status == ASRS_STATUS_ERROR) {
      Serial.print(operationName);
      Serial.print(F(" failed: "));
      Serial.println(asrsErrorName(status.error));
      return false;
    }
  }

  Serial.print(operationName);
  Serial.println(F(" ended before a terminal status was observed."));
  return false;
}

bool MoveAndWait(int32_t targetX, int32_t targetZ) {
  if (!master.sendTravelCommand(targetX, targetZ)) {
    Serial.print(F("Move command was not accepted: "));
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print(F("Move command accepted. Target X = "));
  Serial.print(targetX);
  Serial.print(F(", target Z = "));
  Serial.println(targetZ);

  if (!waitForAcceptedOperationDone("Move", MOVE_WAIT_TIMEOUT_MS)) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the slave has reached targetX,targetZ.
  TravelX += 200;
  TravelZ += 200;
  return true;
}

bool HomeAndWait(bool xHome, bool zHome) {
  if (!master.sendHomingCommand(xHome, zHome)) {
    Serial.print(F("Homing command was not accepted: "));
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print(F("Homing command accepted. X home = "));
  Serial.print(xHome ? F("true") : F("false"));
  Serial.print(F(", Z home = "));
  Serial.println(zHome ? F("true") : F("false"));

  if (!waitForAcceptedOperationDone("Homing", HOMING_WAIT_TIMEOUT_MS)) {
    return false;
  }

  // USER CODE SECTION:
  // Add code here when the requested homing operation is complete.

  return true;
}

bool ReadCoordinates(ASRS_Coordinates &coordinates) {
  if (!master.requestCoordinates(coordinates)) {
    Serial.print(F("Coordinate request failed: "));
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print(F("Coordinates received: X = "));
  Serial.print(coordinates.x);
  Serial.print(F(", Z = "));
  Serial.println(coordinates.z);

  // USER CODE SECTION:
  // Use coordinates.x and coordinates.z here.

  return true;
}

bool ReadLimitSwitches(ASRS_LimitSwitches &limits) {
  if (!master.requestLimitSwitches(limits)) {
    Serial.print(F("Limit-switch request failed: "));
    Serial.println(asrsErrorName(master.lastError()));
    return false;
  }

  Serial.print(F("Limit switches: X-min = "));
  Serial.print(limits.xMinimum ? F("active") : F("inactive"));
  Serial.print(F(", X-max = "));
  Serial.print(limits.xMaximum ? F("active") : F("inactive"));
  Serial.print(F(", Z-min = "));
  Serial.print(limits.zMinimum ? F("active") : F("inactive"));
  Serial.print(F(", Z-max = "));
  Serial.println(limits.zMaximum ? F("active") : F("inactive"));

  // USER CODE SECTION:
  // Use limits.xMinimum, limits.xMaximum, limits.zMinimum, and limits.zMaximum here.

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  printTemplateHeader();
  asrsCommunication.begin(ASRS_SOFTSERIAL_BAUD);

  Serial.println(F("ASRS SoftwareSerial link started."));
}

void loop() {
  ReadCoordinates(SlaveCoordinates);
  HomeAndWait(true,true);
  MoveAndWait(TravelX,TravelZ);
  ReadLimitSwitches(SlaveLimitSwitches);
  delay(5000);
}
