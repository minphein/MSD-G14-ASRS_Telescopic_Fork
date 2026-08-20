#ifndef ASRS_ESPNOW_MASTER_SESSION_H
#define ASRS_ESPNOW_MASTER_SESSION_H

#include <Arduino.h>
#include "../ASRS_Comm_ESPNow.h"

#if defined(ESP32)

class ASRS_ESPNow_MasterSession {
public:
  ASRS_ESPNow_MasterSession(ASRS_Comm_ESPNow &communication);

  bool begin(uint8_t channel = 1,
             uint32_t initialPairingTimeoutMs = 10000,
             Stream *diagnosticStream = nullptr);
  void update();
  void recordPeerActivity();

  bool connected() const;
  bool ready() const;
  bool pairing() const;
  ASRS_Error lastError() const;
  ASRS_Comm_ESPNow &communication();

  void setRepairingTimeout(uint32_t timeoutMs);
  void setLinkTimeout(uint32_t timeoutMs);
  void setHeartbeatInterval(uint32_t intervalMs);

private:
  enum State : uint8_t {
    STATE_NOT_STARTED = 0,
    STATE_PAIRING = 1,
    STATE_CONNECTED = 2
  };

  ASRS_Comm_ESPNow &_communication;
  Stream *_diagnosticStream;
  State _state;
  uint32_t _repairingTimeoutMs;
  uint32_t _linkTimeoutMs;
  uint32_t _heartbeatIntervalMs;
  uint32_t _lastHeartbeatMs;
  uint32_t _lastPeerActivityMs;
  bool _linkLossPrinted;

  bool pairWithSlave(uint32_t timeoutMs);
  void sendHeartbeatIfDue();
  void recordTransportActivity();
  bool hasLinkTimedOut() const;
  void recoverPairingAfterLinkLoss();
  void printPeerAddress(const char *label);
  void printDiagnostic(const char *message);
};

#endif

#endif
