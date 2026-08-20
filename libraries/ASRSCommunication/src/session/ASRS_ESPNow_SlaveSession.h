#ifndef ASRS_ESPNOW_SLAVE_SESSION_H
#define ASRS_ESPNOW_SLAVE_SESSION_H

#include <Arduino.h>
#include "../ASRS_Comm_ESPNow.h"

#if defined(ESP32)

class ASRS_ESPNow_SlaveSession {
public:
  ASRS_ESPNow_SlaveSession(ASRS_Comm_ESPNow &communication);

  bool begin(bool espNowSelected,
             uint8_t channel = 1,
             bool rememberPeer = false,
             Stream *diagnosticStream = nullptr);
  void update();
  void recordPeerActivity();

  bool connected() const;
  bool ready() const;
  bool enabled() const;
  ASRS_Error lastError() const;
  ASRS_Comm_ESPNow &communication();

  void setLinkTimeout(uint32_t timeoutMs);

private:
  ASRS_Comm_ESPNow &_communication;
  Stream *_diagnosticStream;
  bool _enabled;
  bool _pairingPrinted;
  bool _linkLossPrinted;
  uint32_t _linkTimeoutMs;
  uint32_t _lastPeerActivityMs;

  void recordTransportActivity();
  bool hasLinkTimedOut() const;
  void printPeerAddress(const char *label);
};

#endif

#endif
