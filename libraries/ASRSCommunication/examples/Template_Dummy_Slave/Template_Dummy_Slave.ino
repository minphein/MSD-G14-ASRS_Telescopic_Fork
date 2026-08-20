#include <Arduino.h>
#include <ASRSCommunication.h>

#if !defined(ESP32)
#error "The template dummy slave requires an ESP32-compatible board."
#endif

static constexpr uint8_t ASRS_ESPNOW_CHANNEL = 1;
static constexpr uint32_t ASRS_UART_BAUD = 9600;
static constexpr int8_t ASRS_UART_RX_PIN = 18;
static constexpr int8_t ASRS_UART_TX_PIN = 17;
static constexpr uint8_t ASRS_MODE_SELECT_PIN = 2;
static constexpr uint8_t ASRS_MODE_ESPNOW_LEVEL = HIGH;
static constexpr uint32_t BUSY_REPORT_DELAY_MS = 100;

// Dummy data parameters. Change these values to simulate different slave states.
int32_t dummyCurrentX = 0;
int32_t dummyCurrentZ = 0;
bool dummyXMinimumLimit = false;
bool dummyXMaximumLimit = false;
bool dummyZMinimumLimit = false;
bool dummyZMaximumLimit = false;
uint32_t dummyMoveDurationMs = 3000;
uint32_t dummyHomeDurationMs = 2000;
bool dummyForceMoveError = false;
bool dummyForceHomeError = false;
ASRS_Error dummyForcedError = ASRS_ERROR_LIMIT_REACHED;

ASRS_Comm_UART uartCommunication(Serial1);
ASRS_Comm_ESPNow espNowCommunication;
ASRS_ESPNow_SlaveSession espNowSession(espNowCommunication);
ASRS_Slave uartDummySlave(uartCommunication);
ASRS_Slave espNowDummySlave(espNowCommunication);
ASRS_Slave *selectedDummySlave = nullptr;
ASRS_Comm_Base *selectedCommunication = nullptr;

enum DummyOperationType : uint8_t {
  DUMMY_OPERATION_NONE,
  DUMMY_OPERATION_MOVE,
  DUMMY_OPERATION_HOME
};

static bool useEspNow = false;
static DummyOperationType dummyOperation = DUMMY_OPERATION_NONE;
static uint32_t operationAcceptedMs = 0;
static uint32_t operationStartedMs = 0;
static bool busyReported = false;
static int32_t pendingTargetX = 0;
static int32_t pendingTargetZ = 0;
static bool pendingHomeX = false;
static bool pendingHomeZ = false;

static void publishDummyState() {
  if (selectedDummySlave == nullptr) {
    return;
  }

  selectedDummySlave->setCoordinates(dummyCurrentX, dummyCurrentZ);
  selectedDummySlave->setLimitSwitches(
      dummyXMinimumLimit,
      dummyXMaximumLimit,
      dummyZMinimumLimit,
      dummyZMaximumLimit);
}

static void printDummyState() {
  Serial.print("Dummy coordinates: X = ");
  Serial.print(dummyCurrentX);
  Serial.print(", Z = ");
  Serial.println(dummyCurrentZ);
  Serial.print("Dummy limits: X-min = ");
  Serial.print(dummyXMinimumLimit ? "active" : "inactive");
  Serial.print(", X-max = ");
  Serial.print(dummyXMaximumLimit ? "active" : "inactive");
  Serial.print(", Z-min = ");
  Serial.print(dummyZMinimumLimit ? "active" : "inactive");
  Serial.print(", Z-max = ");
  Serial.println(dummyZMaximumLimit ? "active" : "inactive");
}

static void printSelectedMode() {
  Serial.print("GPIO2 communication-select level: ");
  Serial.println(useEspNow ? "HIGH, ESP-NOW selected" : "LOW, UART selected");
  Serial.println("UART default pins: TX = GPIO17, RX = GPIO18, baud = 9600.");
  Serial.println("GPIO2 HIGH selects ESP-NOW. GPIO2 LOW selects UART.");
}

static void updateSelectedCommunication() {
  if (selectedDummySlave == nullptr) {
    return;
  }

  if (useEspNow) {
    espNowSession.update();

    if (!espNowSession.connected()) {
      return;
    }

    if (espNowCommunication.available()) {
      espNowSession.recordPeerActivity();
    }
  }

  selectedDummySlave->update();
}

static void checkAcceptedMoveCommand() {
  if (selectedDummySlave == nullptr ||
      dummyOperation != DUMMY_OPERATION_NONE) {
    return;
  }

  ASRS_TravelCommand command;
  if (!selectedDummySlave->readTravelCommand(command)) {
    return;
  }

  pendingTargetX = command.xDistance;
  pendingTargetZ = command.zDistance;
  dummyOperation = DUMMY_OPERATION_MOVE;
  operationAcceptedMs = millis();
  busyReported = false;

  Serial.print("Dummy move accepted. Target X = ");
  Serial.print(pendingTargetX);
  Serial.print(", target Z = ");
  Serial.println(pendingTargetZ);
}

static void checkAcceptedHomeCommand() {
  if (selectedDummySlave == nullptr ||
      dummyOperation != DUMMY_OPERATION_NONE) {
    return;
  }

  ASRS_HomingCommand command;
  if (!selectedDummySlave->readHomingCommand(command)) {
    return;
  }

  pendingHomeX = command.xHome;
  pendingHomeZ = command.zHome;
  dummyOperation = DUMMY_OPERATION_HOME;
  operationAcceptedMs = millis();
  busyReported = false;

  Serial.print("Dummy homing accepted. X home = ");
  Serial.print(pendingHomeX ? "true" : "false");
  Serial.print(", Z home = ");
  Serial.println(pendingHomeZ ? "true" : "false");
}

static void updateDummyOperation() {
  if (selectedDummySlave == nullptr ||
      dummyOperation == DUMMY_OPERATION_NONE) {
    return;
  }

  if (!busyReported) {
    if (millis() - operationAcceptedMs < BUSY_REPORT_DELAY_MS) {
      return;
    }

    if (!selectedDummySlave->reportOperationBusy()) {
      return;
    }

    busyReported = true;
    operationStartedMs = millis();
    Serial.println("Dummy operation reported BUSY.");
    return;
  }

  const uint32_t requiredDurationMs =
      dummyOperation == DUMMY_OPERATION_MOVE ? dummyMoveDurationMs : dummyHomeDurationMs;
  if (millis() - operationStartedMs < requiredDurationMs) {
    return;
  }

  if (dummyOperation == DUMMY_OPERATION_MOVE && dummyForceMoveError) {
    selectedDummySlave->failActiveOperation(dummyForcedError);
    Serial.print("Dummy move forced ERROR: ");
    Serial.println(asrsErrorName(dummyForcedError));
    dummyOperation = DUMMY_OPERATION_NONE;
    return;
  }

  if (dummyOperation == DUMMY_OPERATION_HOME && dummyForceHomeError) {
    selectedDummySlave->failActiveOperation(dummyForcedError);
    Serial.print("Dummy homing forced ERROR: ");
    Serial.println(asrsErrorName(dummyForcedError));
    dummyOperation = DUMMY_OPERATION_NONE;
    return;
  }

  if (dummyOperation == DUMMY_OPERATION_MOVE) {
    dummyCurrentX = pendingTargetX;
    dummyCurrentZ = pendingTargetZ;
    dummyXMinimumLimit = false;
    dummyZMinimumLimit = false;
    Serial.println("Dummy move completed.");
  } else {
    if (pendingHomeX) {
      dummyCurrentX = 0;
      dummyXMinimumLimit = true;
      dummyXMaximumLimit = false;
    }
    if (pendingHomeZ) {
      dummyCurrentZ = 0;
      dummyZMinimumLimit = true;
      dummyZMaximumLimit = false;
    }
    Serial.println("Dummy homing completed.");
  }

  publishDummyState();
  if (!selectedDummySlave->completeActiveOperation()) {
    Serial.println("Failed to publish dummy DONE status.");
    return;
  }

  printDummyState();
  dummyOperation = DUMMY_OPERATION_NONE;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ASRS_MODE_SELECT_PIN, INPUT_PULLUP);
  useEspNow = digitalRead(ASRS_MODE_SELECT_PIN) == ASRS_MODE_ESPNOW_LEVEL;
  uartCommunication.begin(ASRS_UART_BAUD, ASRS_UART_RX_PIN, ASRS_UART_TX_PIN);

  if (useEspNow) {
    selectedCommunication = &espNowCommunication;
    selectedDummySlave = &espNowDummySlave;

    if (!espNowSession.begin(true, ASRS_ESPNOW_CHANNEL, false, &Serial)) {
      Serial.print("ESP-NOW template slave initialisation failed: ");
      Serial.println(asrsErrorName(espNowSession.lastError()));
      selectedCommunication = nullptr;
      selectedDummySlave = nullptr;
      return;
    }
  } else {
    selectedCommunication = &uartCommunication;
    selectedDummySlave = &uartDummySlave;
  }

  publishDummyState();
  selectedDummySlave->setOperationStatus(ASRS_STATUS_IDLE);

  Serial.println("ASRS template dummy slave started.");
  printSelectedMode();
  Serial.println("Edit the global dummy parameters at the top of this file to simulate data and faults.");
  printDummyState();
}

void loop() {
  updateSelectedCommunication();
  checkAcceptedMoveCommand();
  checkAcceptedHomeCommand();
  updateDummyOperation();
}
