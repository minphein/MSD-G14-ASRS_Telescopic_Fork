#include "ASRS_ESPNow_SlaveSession.h"

#if defined(ESP32)

ASRS_ESPNow_SlaveSession::ASRS_ESPNow_SlaveSession(ASRS_Comm_ESPNow &communication)
  : _communication(communication),
    _diagnosticStream(nullptr),
    _enabled(false),
    _pairingPrinted(false),
    _linkLossPrinted(false),
    _linkTimeoutMs(6000),
    _lastPeerActivityMs(0) {
}

bool ASRS_ESPNow_SlaveSession::begin(bool espNowSelected,
                                     uint8_t channel,
                                     bool rememberPeer,
                                     Stream *diagnosticStream) {
  _diagnosticStream = diagnosticStream;
  _enabled = false;
  _pairingPrinted = false;
  _linkLossPrinted = false;
  _lastPeerActivityMs = 0;

  if (!espNowSelected) {
    if (_diagnosticStream != nullptr) {
      _diagnosticStream->println("DIP switch does not select ESP-NOW. Pairing is disabled.");
    }
    return false;
  }

  if (!_communication.beginPairingSlave(true, channel, rememberPeer)) {
    if (_diagnosticStream != nullptr) {
      _diagnosticStream->print("ESP-NOW slave initialisation failed: ");
      _diagnosticStream->println(asrsErrorName(_communication.lastError()));
    }
    return false;
  }

  _enabled = true;
  if (_diagnosticStream != nullptr) {
    _diagnosticStream->println("Pairing rule: accept the first valid pair request while ESP-NOW is selected.");
    if (rememberPeer) {
      _diagnosticStream->println("The paired master MAC address may be restored from ESP32 non-volatile storage.");
    } else {
      _diagnosticStream->println("The paired master MAC address is held in RAM only and is cleared by resetting the slave.");
    }
  }

  return true;
}

void ASRS_ESPNow_SlaveSession::update() {
  if (!_enabled) {
    return;
  }

  if (_communication.isPaired() && !_pairingPrinted) {
    printPeerAddress("Paired master MAC address: ");
    recordPeerActivity();
    _pairingPrinted = true;
  }

  recordTransportActivity();

  if (hasLinkTimedOut() && !_linkLossPrinted && _diagnosticStream != nullptr) {
    _diagnosticStream->print("Warning: ESP-NOW link timeout. No packet from paired master for ");
    _diagnosticStream->print(millis() - _lastPeerActivityMs);
    _diagnosticStream->print(" ms. Consecutive delivery failures = ");
    _diagnosticStream->println(_communication.consecutiveDeliveryFailures());
    _linkLossPrinted = true;
  }
}

void ASRS_ESPNow_SlaveSession::recordPeerActivity() {
  _lastPeerActivityMs = millis();
  _linkLossPrinted = false;
}

bool ASRS_ESPNow_SlaveSession::connected() const {
  return _enabled && _communication.isPaired();
}

bool ASRS_ESPNow_SlaveSession::ready() const {
  return connected();
}

bool ASRS_ESPNow_SlaveSession::enabled() const {
  return _enabled;
}

ASRS_Error ASRS_ESPNow_SlaveSession::lastError() const {
  return _communication.lastError();
}

ASRS_Comm_ESPNow &ASRS_ESPNow_SlaveSession::communication() {
  return _communication;
}

void ASRS_ESPNow_SlaveSession::setLinkTimeout(uint32_t timeoutMs) {
  _linkTimeoutMs = timeoutMs;
}

void ASRS_ESPNow_SlaveSession::recordTransportActivity() {
  const uint32_t nowMs = millis();
  const uint32_t receiveAgeMs = _communication.millisecondsSinceLastReceive();
  const uint32_t deliveryAgeMs = _communication.millisecondsSinceLastDelivery();
  const bool newReceive =
      receiveAgeMs != UINT32_MAX &&
      nowMs - receiveAgeMs > _lastPeerActivityMs;
  const bool newDelivery =
      deliveryAgeMs != UINT32_MAX &&
      nowMs - deliveryAgeMs > _lastPeerActivityMs;

  if (newReceive || newDelivery) {
    recordPeerActivity();
  }
}

bool ASRS_ESPNow_SlaveSession::hasLinkTimedOut() const {
  if (_linkTimeoutMs == 0 || !_communication.isPaired() || _lastPeerActivityMs == 0) {
    return false;
  }

  const uint32_t elapsedSincePeerActivity = millis() - _lastPeerActivityMs;
  return elapsedSincePeerActivity > _linkTimeoutMs || _communication.linkTimedOut(_linkTimeoutMs);
}

void ASRS_ESPNow_SlaveSession::printPeerAddress(const char *label) {
  if (_diagnosticStream == nullptr) {
    return;
  }

  uint8_t peerAddress[6] = {};
  _communication.copyPeerAddress(peerAddress);
  _diagnosticStream->print(label);
  for (uint8_t i = 0; i < 6; ++i) {
    if (i > 0) {
      _diagnosticStream->print(":");
    }
    if (peerAddress[i] < 0x10) {
      _diagnosticStream->print("0");
    }
    _diagnosticStream->print(peerAddress[i], HEX);
  }
  _diagnosticStream->println();
}

#endif
