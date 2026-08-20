#include <Arduino.h>
#include <ASRSCommunication.h>

// INTERFACE REFERENCE ONLY.
// The ASRS tower firmware is maintained by another developer. Do not upload
// this sketch to the tower without that developer's explicit authorization.

#if !defined(ESP32)
#error "ASRS_Tower_Slave requires an ESP32-compatible tower controller."
#endif

constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr bool REMEMBER_MASTER = true;
constexpr bool PAIRING_ALLOWED_AT_BOOT = true;

ASRS_Comm_ESPNow communication;
ASRS_ESPNow_SlaveSession session(communication);
ASRS_Slave tower(communication);

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!session.begin(PAIRING_ALLOWED_AT_BOOT,
                     ESPNOW_CHANNEL,
                     REMEMBER_MASTER,
                     &Serial)) {
    Serial.print("Tower ESP-NOW initialization failed: ");
    Serial.println(asrsErrorName(session.lastError()));
    return;
  }

  // Uses the existing ASRS_Slave/ASRSMotion integration. The companion
  // ASRSMotion library and the tower's limit/sensor wiring must be installed.
  tower.setHomePosition(500, 400);
  tower.enableMotionControl(true);
  tower.beginTofCoordinateSensors();
  tower.setOperationStatus(ASRS_STATUS_IDLE);

  Serial.println("ASRS tower slave ready for pairing and X/Z commands.");
}

void loop() {
  session.update();
  if (communication.available()) session.recordPeerActivity();
  tower.update();
}
