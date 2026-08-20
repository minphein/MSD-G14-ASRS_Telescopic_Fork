#include "ASRS_Comm_ESPNow.h"

#if defined(ESP32)
#include <esp_arduino_version.h>

ASRS_Comm_ESPNow *ASRS_Comm_ESPNow::_instance = nullptr;
ASRS_Comm_ESPNow::RawReceiveHandler ASRS_Comm_ESPNow::_rawReceiveHandler = nullptr;
ASRS_Packet ASRS_Comm_ESPNow::_pendingPacket = {};
bool ASRS_Comm_ESPNow::_hasPendingPacket = false;
ASRS_Error ASRS_Comm_ESPNow::_staticError = ASRS_ERROR_NONE;
const uint8_t ASRS_Comm_ESPNow::BROADCAST_ADDRESS[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void asrsEspNowReceiveCallback(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  const uint8_t *senderMac = info == nullptr ? nullptr : info->src_addr;
  ASRS_Comm_ESPNow::processReceivedBytes(senderMac, data, length);
}
#else
static void asrsEspNowReceiveCallback(const uint8_t *mac, const uint8_t *data, int length) {
  ASRS_Comm_ESPNow::processReceivedBytes(mac, data, length);
}
#endif

#if ESP_ARDUINO_VERSION_MAJOR > 3 || \
    (ESP_ARDUINO_VERSION_MAJOR == 3 && ESP_ARDUINO_VERSION_MINOR >= 3)
static void asrsEspNowSendCallback(const esp_now_send_info_t *info,
                                   esp_now_send_status_t status) {
  const uint8_t *destinationMac = info == nullptr ? nullptr : info->des_addr;
  ASRS_Comm_ESPNow::processSendStatus(destinationMac, status);
}
#else
static void asrsEspNowSendCallback(const uint8_t *mac,
                                   esp_now_send_status_t status) {
  ASRS_Comm_ESPNow::processSendStatus(mac, status);
}
#endif

ASRS_Comm_ESPNow::ASRS_Comm_ESPNow()
  : _sequenceCounter(0),
    _lastError(ASRS_ERROR_NONE),
    _ready(false),
    _paired(false),
    _pairingAllowed(false),
    _rememberPeer(false),
    _storePeerPending(false),
    _lastSendDelivered(false),
    _sendStatusAvailable(false),
    _consecutiveDeliveryFailures(0),
    _lastDeliveryMs(0),
    _channel(1),
    _pairingRole(PAIRING_ROLE_FIXED_PEER),
    _lastReceiveMs(0) {
  memset(_peerAddress, 0xFF, sizeof(_peerAddress));
}

bool ASRS_Comm_ESPNow::begin(const uint8_t peerAddress[6], uint8_t channel) {
  setPeerAddress(peerAddress);
  _pairingRole = PAIRING_ROLE_FIXED_PEER;
  _pairingAllowed = false;
  _rememberPeer = false;
  _storePeerPending = false;
  _paired = !isBroadcastAddress(_peerAddress);

  if (!initialiseEspNow(channel)) {
    return false;
  }

  if (!addPeer(_peerAddress)) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  _ready = true;
  _lastReceiveMs = 0;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::beginPairingMaster(uint8_t channel) {
  setPeerAddress(BROADCAST_ADDRESS);
  _pairingRole = PAIRING_ROLE_MASTER;
  _pairingAllowed = true;
  _rememberPeer = false;
  _storePeerPending = false;
  _paired = false;

  if (!initialiseEspNow(channel)) {
    return false;
  }

  if (!addPeer(BROADCAST_ADDRESS)) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  _ready = true;
  _lastReceiveMs = 0;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::beginPairingSlave(bool pairingAllowed, uint8_t channel, bool rememberPeer) {
  setPeerAddress(BROADCAST_ADDRESS);
  _pairingRole = PAIRING_ROLE_SLAVE;
  _pairingAllowed = pairingAllowed;
  _rememberPeer = rememberPeer;
  _storePeerPending = false;
  _paired = false;

  if (!initialiseEspNow(channel)) {
    return false;
  }

  if (_rememberPeer && loadStoredPeer()) {
    _paired = true;
    _pairingAllowed = false;
    if (!addPeer(_peerAddress)) {
      _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
      return false;
    }
  }

  _ready = true;
  _lastReceiveMs = 0;
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::initialiseEspNow(uint8_t channel) {
  _channel = channel;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  //initialize the espNow
  const esp_err_t initResult = esp_now_init();
  if (initResult != ESP_OK) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    //return false if the 
    return false;
  }

  _instance = this;
  esp_now_register_recv_cb(asrsEspNowReceiveCallback);
  esp_now_register_send_cb(asrsEspNowSendCallback);
  return true;
}

bool ASRS_Comm_ESPNow::addPeer(const uint8_t peerAddress[6]) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerAddress, sizeof(peer.peer_addr));
  peer.channel = _channel;
  peer.encrypt = false;

  return esp_now_is_peer_exist(peerAddress) || esp_now_add_peer(&peer) == ESP_OK;
}

bool ASRS_Comm_ESPNow::pairAsMaster(uint32_t timeoutMs, uint32_t retryIntervalMs) {
  if (!_ready || _pairingRole != PAIRING_ROLE_MASTER) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  const uint32_t startTime = millis();
  uint32_t lastAttemptTime = 0;

  while (!_paired && millis() - startTime < timeoutMs) {
    const uint32_t now = millis();
    if (lastAttemptTime == 0 || now - lastAttemptTime >= retryIntervalMs) {
      lastAttemptTime = now;
      sendPairingPacket(ASRS_CMD_PAIR_REQUEST, BROADCAST_ADDRESS);
    }
    delay(10);
  }

  if (!_paired) {
    _lastError = ASRS_ERROR_TIMEOUT;
    return false;
  }

  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::restartPairingMaster() {
  if (!_ready || _pairingRole != PAIRING_ROLE_MASTER) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  setPeerAddress(BROADCAST_ADDRESS);
  _paired = false;
  _pairingAllowed = true;
  resetDeliveryState();

  if (!addPeer(BROADCAST_ADDRESS)) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::clearStoredPeer() {
  Preferences preferences;
  if (!preferences.begin("asrs_espnow", false)) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  preferences.remove("peer_valid");
  preferences.remove("peer_mac");
  preferences.end();
  _lastError = ASRS_ERROR_NONE;
  return true;
}

bool ASRS_Comm_ESPNow::isPaired() const {
  return _paired;
}

void ASRS_Comm_ESPNow::copyPeerAddress(uint8_t destination[6]) const {
  memcpy(destination, _peerAddress, sizeof(_peerAddress));
}

bool ASRS_Comm_ESPNow::lastSendDelivered() const {
  return _sendStatusAvailable && _lastSendDelivered;
}

uint8_t ASRS_Comm_ESPNow::consecutiveDeliveryFailures() const {
  return _consecutiveDeliveryFailures;
}

uint32_t ASRS_Comm_ESPNow::millisecondsSinceLastDelivery() const {
  if (_lastDeliveryMs == 0) {
    return UINT32_MAX;
  }

  return millis() - _lastDeliveryMs;
}

uint32_t ASRS_Comm_ESPNow::millisecondsSinceLastReceive() const {
  if (_lastReceiveMs == 0) {
    return UINT32_MAX;
  }

  return millis() - _lastReceiveMs;
}

bool ASRS_Comm_ESPNow::linkTimedOut(uint32_t timeoutMs) const {
  if (_lastDeliveryMs == 0 && _lastReceiveMs == 0) {
    return false;
  }

  return _paired &&
         millisecondsSinceLastDelivery() > timeoutMs &&
         millisecondsSinceLastReceive() > timeoutMs;
}

bool ASRS_Comm_ESPNow::sendHeartbeat() {
  savePendingPeerIfNeeded();

  if (!_ready || !_paired) {
    _lastError = ASRS_ERROR_TIMEOUT;
    return false;
  }

  return sendControlPacket(ASRS_CMD_HEARTBEAT, _peerAddress);
}

void ASRS_Comm_ESPNow::setRawReceiveHandler(RawReceiveHandler handler) {
  _rawReceiveHandler = handler;
}

bool ASRS_Comm_ESPNow::sendPacket(ASRS_Packet &packet) {
  savePendingPeerIfNeeded();

  if (!_ready) {
    _lastError = ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
    return false;
  }

  if (packet.len > ASRS_MAX_DATA) {
    _lastError = ASRS_ERROR_PAYLOAD_LENGTH;
    return false;
  }

  if ((_pairingRole == PAIRING_ROLE_MASTER || _pairingRole == PAIRING_ROLE_SLAVE) && !_paired) {
    _lastError = ASRS_ERROR_TIMEOUT;
    return false;
  }

  if (packet.seq == 0) {
    packet.seq = _sequenceCounter++;
  }
  //The ESPNow working by sending packet in packet.
  //However, still encode it is to make the multiple medium share same packet format.
  //future compatibility, easier testing and application level error detection.
  uint8_t frame[ASRS_FrameCodec::MAX_FRAME_SIZE] = {};
  const size_t frameLength = ASRS_FrameCodec::encode(frame, packet);
  const bool sent = esp_now_send(_peerAddress, frame, frameLength) == ESP_OK;
  _lastError = sent ? ASRS_ERROR_NONE : ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
  return sent;
}

bool ASRS_Comm_ESPNow::available() {
  savePendingPeerIfNeeded();
  return _hasPendingPacket;
}

bool ASRS_Comm_ESPNow::readPacket(ASRS_Packet &packet) {
  savePendingPeerIfNeeded();

  if (!_hasPendingPacket) {
    return false;
  }

  packet = _pendingPacket;
  _hasPendingPacket = false;
  return true;
}

ASRS_Medium ASRS_Comm_ESPNow::medium() const {
  return ASRS_MEDIUM_ESPNOW;
}

ASRS_Error ASRS_Comm_ESPNow::lastError() const {
  return _staticError != ASRS_ERROR_NONE ? _staticError : _lastError;
}

void ASRS_Comm_ESPNow::processReceivedBytes(const uint8_t *data, int length) {
  processReceivedBytes(nullptr, data, length);
}

void ASRS_Comm_ESPNow::processReceivedBytes(const uint8_t *senderMac, const uint8_t *data, int length) {
  if (_rawReceiveHandler != nullptr &&
      _rawReceiveHandler(senderMac, data, length)) {
    return;
  }

  if (_instance != nullptr) {
    _instance->handleReceivedBytes(senderMac, data, length);
    return;
  }

  if (data == nullptr || length <= 0) {
    _staticError = ASRS_ERROR_INVALID_FRAME;
    return;
  }

  ASRS_Error error = ASRS_ERROR_NONE;
  if (ASRS_FrameCodec::decode(data, static_cast<size_t>(length), _pendingPacket, error)) {
    _hasPendingPacket = true;
    _staticError = ASRS_ERROR_NONE;
  } else {
    _staticError = error;
  }
}

void ASRS_Comm_ESPNow::processSendStatus(const uint8_t *peerMac, esp_now_send_status_t status) {
  if (_instance != nullptr) {
    _instance->handleSendStatus(peerMac, status);
  }
}

bool ASRS_Comm_ESPNow::sendPairingPacket(uint8_t command, const uint8_t destination[6]) {
  return sendControlPacket(command, destination);
}

bool ASRS_Comm_ESPNow::sendControlPacket(uint8_t command, const uint8_t destination[6]) {
  ASRS_Packet packet = {};
  asrsClearPacket(packet);
  packet.cmd = command;
  packet.seq = _sequenceCounter++;

  if (command == ASRS_CMD_PAIR_REQUEST || command == ASRS_CMD_PAIR_ACK) {
    packet.len = 4;
    packet.data[0] = 'A';
    packet.data[1] = 'S';
    packet.data[2] = 'R';
    packet.data[3] = 'S';
  }

  uint8_t frame[ASRS_FrameCodec::MAX_FRAME_SIZE] = {};
  const size_t frameLength = ASRS_FrameCodec::encode(frame, packet);
  const bool sent = esp_now_send(destination, frame, frameLength) == ESP_OK;
  _lastError = sent ? ASRS_ERROR_NONE : ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
  return sent;
}

bool ASRS_Comm_ESPNow::loadStoredPeer() {
  Preferences preferences;
  if (!preferences.begin("asrs_espnow", false)) {
    return false;
  }

  const bool valid = preferences.getBool("peer_valid", false);
  const size_t length = preferences.getBytes("peer_mac", _peerAddress, sizeof(_peerAddress));
  preferences.end();
  return valid && length == sizeof(_peerAddress) && !isBroadcastAddress(_peerAddress);
}

bool ASRS_Comm_ESPNow::saveStoredPeer() {
  Preferences preferences;
  if (!preferences.begin("asrs_espnow", false)) {
    return false;
  }

  const size_t length = preferences.putBytes("peer_mac", _peerAddress, sizeof(_peerAddress));
  preferences.putBool("peer_valid", length == sizeof(_peerAddress));
  preferences.end();
  return length == sizeof(_peerAddress);
}

void ASRS_Comm_ESPNow::savePendingPeerIfNeeded() {
  if (!_storePeerPending) {
    return;
  }

  if (saveStoredPeer()) {
    _storePeerPending = false;
  }
}

void ASRS_Comm_ESPNow::resetDeliveryState() {
  _lastSendDelivered = false;
  _sendStatusAvailable = false;
  _consecutiveDeliveryFailures = 0;
  _lastDeliveryMs = 0;
  _lastReceiveMs = 0;
}

bool ASRS_Comm_ESPNow::isBroadcastAddress(const uint8_t address[6]) const {
  return memcmp(address, BROADCAST_ADDRESS, sizeof(_peerAddress)) == 0;
}

bool ASRS_Comm_ESPNow::isKnownPeer(const uint8_t senderMac[6]) const {
  return senderMac != nullptr && memcmp(senderMac, _peerAddress, sizeof(_peerAddress)) == 0;
}

void ASRS_Comm_ESPNow::setPeerAddress(const uint8_t peerAddress[6]) {
  memcpy(_peerAddress, peerAddress, sizeof(_peerAddress));
}

void ASRS_Comm_ESPNow::handleReceivedBytes(const uint8_t *senderMac, const uint8_t *data, int length) {
  if (data == nullptr || length <= 0) {
    _staticError = ASRS_ERROR_INVALID_FRAME;
    _lastError = _staticError;
    return;
  }

  ASRS_Packet packet = {};
  ASRS_Error error = ASRS_ERROR_NONE;
  if (!ASRS_FrameCodec::decode(data, static_cast<size_t>(length), packet, error)) {
    _staticError = error;
    _lastError = error;
    return;
  }

  const bool isPairingPacket = packet.cmd == ASRS_CMD_PAIR_REQUEST || packet.cmd == ASRS_CMD_PAIR_ACK;
  const bool hasPairingMagic = packet.len == 4 &&
                               packet.data[0] == 'A' &&
                               packet.data[1] == 'S' &&
                               packet.data[2] == 'R' &&
                               packet.data[3] == 'S';

  if (isPairingPacket) {
    if (!hasPairingMagic || senderMac == nullptr) {
      _staticError = ASRS_ERROR_INVALID_FRAME;
      _lastError = _staticError;
      return;
    }

    if (_pairingRole == PAIRING_ROLE_SLAVE && packet.cmd == ASRS_CMD_PAIR_REQUEST) {
      if (!_paired && _pairingAllowed) {
        setPeerAddress(senderMac);
        _paired = addPeer(_peerAddress);
        _pairingAllowed = false;
        if (_paired && _rememberPeer) {
          _storePeerPending = true;
        }
      }

      if (_paired && isKnownPeer(senderMac)) {
        sendPairingPacket(ASRS_CMD_PAIR_ACK, _peerAddress);
        _lastReceiveMs = millis();
        _staticError = ASRS_ERROR_NONE;
        _lastError = ASRS_ERROR_NONE;
        return;
      }
    }

    if (_pairingRole == PAIRING_ROLE_MASTER && packet.cmd == ASRS_CMD_PAIR_ACK) {
      setPeerAddress(senderMac);
      _paired = addPeer(_peerAddress);
      resetDeliveryState();
      _lastReceiveMs = millis();
      _staticError = _paired ? ASRS_ERROR_NONE : ASRS_ERROR_ESPNOW_NOT_AVAILABLE;
      _lastError = _staticError;
      return;
    }

    return;
  }

  if ((_pairingRole == PAIRING_ROLE_MASTER || _pairingRole == PAIRING_ROLE_SLAVE) &&
      (!_paired || !isKnownPeer(senderMac))) {
    _staticError = ASRS_ERROR_WRONG_MODE_SELECTED;
    _lastError = _staticError;
    return;
  }

  _lastReceiveMs = millis();

  if (handleControlPacket(packet, senderMac)) {
    return;
  }

  _pendingPacket = packet;
  _hasPendingPacket = true;
  _staticError = ASRS_ERROR_NONE;
  _lastError = ASRS_ERROR_NONE;
}

void ASRS_Comm_ESPNow::handleSendStatus(const uint8_t *peerMac, esp_now_send_status_t status) {
  if (peerMac == nullptr) {
    return;
  }

  const bool relevantPeer = isKnownPeer(peerMac) || isBroadcastAddress(peerMac);
  if (!relevantPeer) {
    return;
  }

  _sendStatusAvailable = true;
  _lastSendDelivered = status == ESP_NOW_SEND_SUCCESS;

  if (_lastSendDelivered) {
    _consecutiveDeliveryFailures = 0;
    _lastDeliveryMs = millis();
    return;
  }

  if (_consecutiveDeliveryFailures < UINT8_MAX) {
    ++_consecutiveDeliveryFailures;
  }
  _lastError = ASRS_ERROR_TIMEOUT;
}

bool ASRS_Comm_ESPNow::handleControlPacket(const ASRS_Packet &packet, const uint8_t *senderMac) {
  if (packet.cmd == ASRS_CMD_HEARTBEAT) {
    if (packet.len != 0 || senderMac == nullptr) {
      _staticError = ASRS_ERROR_INVALID_FRAME;
      _lastError = _staticError;
      return true;
    }

    sendControlPacket(ASRS_CMD_HEARTBEAT_ACK, _peerAddress);
    _staticError = ASRS_ERROR_NONE;
    _lastError = ASRS_ERROR_NONE;
    return true;
  }

  if (packet.cmd == ASRS_CMD_HEARTBEAT_ACK) {
    if (packet.len != 0) {
      _staticError = ASRS_ERROR_INVALID_FRAME;
      _lastError = _staticError;
      return true;
    }

    _staticError = ASRS_ERROR_NONE;
    _lastError = ASRS_ERROR_NONE;
    return true;
  }

  return false;
}

#endif
