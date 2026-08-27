#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>

// =====================================================
// Fork access point connection settings
// =====================================================
const char* SETUP_AP_SSID = "Rack-WiFi-Setup";
const char* SETUP_AP_PASSWORD = "racksetup";
const char* FORK_AP_SSID = "Fork-WiFi-Setup";
const char* FORK_AP_PASSWORD = "forksetup";
IPAddress RACK_SETUP_IP(192, 168, 11, 1);
IPAddress RACK_SETUP_GATEWAY(192, 168, 11, 1);
IPAddress RACK_SETUP_SUBNET(255, 255, 255, 0);
Preferences wifiPreferences;
String routerSSID;
String routerPassword;
IPAddress forkServerIP;
bool forkAddressKnown = false;
bool rackMDNSStarted = false;

// The router assigns addresses by DHCP. The rack discovers the fork by its
// mDNS hostname, so no router-specific IP address is stored in this sketch.

// =====================================================
// Web server
// =====================================================
WebServer server(80);

// =====================================================
// IR sensor GPIO pins
// =====================================================
const uint8_t SENSOR_PINS[4] = {
  25,  // Slot 1
  26,  // Slot 2
  27,  // Slot 3
  32   // Slot 4
};

// Sensor logic:
// LOW  = box detected
// HIGH = no box detected
const uint8_t BOX_DETECTED_STATE = LOW;

// Current slot states
bool slotOccupied[4] = {
  false,
  false,
  false,
  false
};

// Sensor update timing
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 100;
unsigned long previousSensorUpdate = 0;

// Serial Monitor print timing
const unsigned long SERIAL_PRINT_INTERVAL_MS = 1000;
unsigned long previousSerialPrint = 0;

const unsigned long RACK_REPORT_INTERVAL_MS = 1000;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;
unsigned long previousRackReport = 0;
unsigned long previousWiFiRetry = 0;
wl_status_t previousWiFiStatus = WL_NO_SHIELD;

const uint32_t ESP_NOW_MAGIC = 0x46524B31; // "FRK1"
const uint8_t ESP_NOW_VERSION = 1;
const uint8_t ESP_NOW_RACK_STATUS = 1;
const uint8_t ESP_NOW_STATUS_REQUEST = 2;
const uint8_t ESP_NOW_CHANNEL = 1;

struct __attribute__((packed)) ForkRackNowPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t sequence;
  uint8_t slots[4];
  uint32_t uptimeMS;
};

const uint8_t ESP_NOW_BROADCAST_ADDRESS[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
bool espNowReady = false;
volatile bool forkRequestedStatus = false;
uint16_t espNowSequence = 0;

// =====================================================
// Webpage
// =====================================================
const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">

  <meta
    name="viewport"
    content="width=device-width, initial-scale=1.0"
  >

  <title>Rack Occupancy Monitor</title>

  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      padding: 20px;
      background: #eef2f6;
      font-family: Arial, Helvetica, sans-serif;
      text-align: center;
    }

    .container {
      max-width: 700px;
      margin: auto;
    }

    h1 {
      margin-bottom: 5px;
    }

    .subtitle {
      margin-top: 0;
      color: #555;
    }

    .rack {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 15px;
      margin-top: 25px;
      padding: 18px;
      background: #333;
      border-radius: 15px;
    }

    .slot {
      min-height: 150px;
      padding: 20px;
      border-radius: 10px;
      color: white;
      display: flex;
      flex-direction: column;
      justify-content: center;
      transition: background-color 0.3s;
    }

    .slot-number {
      font-size: 27px;
      font-weight: bold;
      margin-bottom: 10px;
    }

    .slot-status {
      font-size: 21px;
      font-weight: bold;
    }

    .occupied {
      background: #c62828;
    }

    .empty {
      background: #2e7d32;
    }

    .unknown {
      background: #757575;
    }

    .summary {
      margin-top: 20px;
      padding: 15px;
      background: white;
      border-radius: 10px;
      font-size: 19px;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.12);
    }

    #connection {
      margin-top: 12px;
      color: #555;
      font-size: 14px;
    }

    @media (max-width: 500px) {
      .rack {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>

<body>

  <div class="container">

    <h1>Rack Occupancy Monitor</h1>

    <p class="subtitle">
      ESP32 four-slot box detection system
    </p>

    <div class="rack">

      <div id="slot1" class="slot unknown">
        <div class="slot-number">Slot 1</div>
        <div id="status1" class="slot-status">CHECKING</div>
      </div>

      <div id="slot2" class="slot unknown">
        <div class="slot-number">Slot 2</div>
        <div id="status2" class="slot-status">CHECKING</div>
      </div>

      <div id="slot3" class="slot unknown">
        <div class="slot-number">Slot 3</div>
        <div id="status3" class="slot-status">CHECKING</div>
      </div>

      <div id="slot4" class="slot unknown">
        <div class="slot-number">Slot 4</div>
        <div id="status4" class="slot-status">CHECKING</div>
      </div>

    </div>

    <div class="summary">
      Occupied:
      <strong id="occupiedCount">0</strong>

      &nbsp; | &nbsp;

      Empty:
      <strong id="emptyCount">4</strong>
    </div>

    <div id="connection">
      Connecting to ESP32...
    </div>

  </div>

  <script>
    function updateSlot(slotNumber, occupied) {
      const slotElement =
        document.getElementById("slot" + slotNumber);

      const statusElement =
        document.getElementById("status" + slotNumber);

      if (occupied) {
        slotElement.className = "slot occupied";
        statusElement.textContent = "OCCUPIED";
      } else {
        slotElement.className = "slot empty";
        statusElement.textContent = "EMPTY";
      }
    }

    async function updateRackStatus() {
      try {
        const response = await fetch("/status", {
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("ESP32 server error");
        }

        const data = await response.json();

        updateSlot(1, data.slot1);
        updateSlot(2, data.slot2);
        updateSlot(3, data.slot3);
        updateSlot(4, data.slot4);

        document.getElementById(
          "occupiedCount"
        ).textContent = data.occupiedCount;

        document.getElementById(
          "emptyCount"
        ).textContent = data.emptyCount;

        document.getElementById(
          "connection"
        ).textContent =
          "Connected â€” Updated at " +
          new Date().toLocaleTimeString();

      } catch (error) {
        document.getElementById(
          "connection"
        ).textContent =
          "Unable to communicate with ESP32";
      }
    }

    updateRackStatus();

    // Refresh every second
    setInterval(updateRackStatus, 1000);
  </script>

</body>
</html>
)rawliteral";

// =====================================================
// Read one IR sensor with basic filtering
// =====================================================
bool readIRSensor(uint8_t pin) {
  const uint8_t numberOfSamples = 5;
  uint8_t detectedSamples = 0;

  for (uint8_t i = 0; i < numberOfSamples; i++) {
    int sensorValue = digitalRead(pin);

    if (sensorValue == BOX_DETECTED_STATE) {
      detectedSamples++;
    }

    delay(2);
  }

  // At least 3 of 5 samples must indicate detection
  return detectedSamples >= 3;
}

// =====================================================
// Update all four slots
// =====================================================
void updateSensors() {
  for (uint8_t i = 0; i < 4; i++) {
    slotOccupied[i] = readIRSensor(SENSOR_PINS[i]);
  }
}

// =====================================================
// Print slot status to Serial Monitor
// =====================================================
void printSlotStatus() {
  Serial.println("----------------------------");

  for (uint8_t i = 0; i < 4; i++) {
    int rawValue = digitalRead(SENSOR_PINS[i]);

    Serial.print("Slot ");
    Serial.print(i + 1);

    Serial.print(" | GPIO ");
    Serial.print(SENSOR_PINS[i]);

    Serial.print(" | Sensor output: ");
    Serial.print(rawValue);

    Serial.print(" | Status: ");

    if (slotOccupied[i]) {
      Serial.println("OCCUPIED");
    } else {
      Serial.println("EMPTY");
    }
  }
}

// =====================================================
// Main webpage handler
// =====================================================
void handleMainPage() {
  server.send_P(
    200,
    "text/html",
    WEBPAGE
  );
}

// =====================================================
// JSON status handler
// =====================================================
void handleStatus() {
  uint8_t occupiedCount = 0;

  for (uint8_t i = 0; i < 4; i++) {
    if (slotOccupied[i]) {
      occupiedCount++;
    }
  }

  uint8_t emptyCount = 4 - occupiedCount;

  String json = "{";

  json += "\"slot1\":";
  json += slotOccupied[0] ? "true" : "false";

  json += ",\"slot2\":";
  json += slotOccupied[1] ? "true" : "false";

  json += ",\"slot3\":";
  json += slotOccupied[2] ? "true" : "false";

  json += ",\"slot4\":";
  json += slotOccupied[3] ? "true" : "false";

  json += ",\"occupiedCount\":";
  json += String(occupiedCount);

  json += ",\"emptyCount\":";
  json += String(emptyCount);

  json += "}";

  server.sendHeader(
    "Cache-Control",
    "no-cache, no-store, must-revalidate"
  );

  server.send(
    200,
    "application/json",
    json
  );
}

// =====================================================
// Page-not-found handler
// =====================================================
void handleNotFound() {
  server.send(
    404,
    "text/plain",
    "Page not found"
  );
}

void handleWiFiSetup() {
  String page = "<h2>Rack Network</h2><p>The rack is an ESP-NOW sensor slave and does not join a Wi-Fi network.</p>";
  page += "<p>ESP-NOW channel: <b>1</b>. Rack diagnostic AP: <b>Rack-WiFi-Setup</b>.</p><p><a href='/'>Rack diagnostic page</a></p>";
  server.send(200, "text/html", page);
}

void handleSaveWiFi() {
  server.send(405, "text/plain", "Wi-Fi client mode is disabled; rack uses ESP-NOW");
}

void startRackSetupAccessPoint() {
  // ESP-NOW peers use the station interface. Keep STA enabled alongside the
  // diagnostic AP so packets use the same interface as the integrated fork.
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAPConfig(RACK_SETUP_IP, RACK_SETUP_GATEWAY, RACK_SETUP_SUBNET);
  if (WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD, ESP_NOW_CHANNEL, false, 4)) {
    esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.println("Rack setup AP: Rack-WiFi-Setup at http://192.168.11.1/wifi");
  } else {
    Serial.println("ERROR: Rack setup AP failed to start");
  }
}

// =====================================================
// Send the four slot states to the fork web server
// =====================================================
void sendRackStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!forkAddressKnown) {
    forkServerIP = MDNS.queryHost("fork-control", 1500);
    forkAddressKnown = forkServerIP != IPAddress(0, 0, 0, 0);
    if (!forkAddressKnown) {
      Serial.println("Fork hostname not found on router");
      return;
    }
    Serial.print("Fork discovered at: ");
    Serial.println(forkServerIP);
  }

  HTTPClient http;
  http.setConnectTimeout(1000);
  http.setTimeout(1500);
  String updateURL = "http://" + forkServerIP.toString() + "/rack-update";
  http.begin(updateURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "slot1=";
  body += slotOccupied[0] ? "1" : "0";
  body += "&slot2=";
  body += slotOccupied[1] ? "1" : "0";
  body += "&slot3=";
  body += slotOccupied[2] ? "1" : "0";
  body += "&slot4=";
  body += slotOccupied[3] ? "1" : "0";

  int responseCode = http.POST(body);
  if (responseCode == 200) {
    Serial.println("Rack status sent to fork: OK");
  } else {
    Serial.print("Fork update failed, HTTP code: ");
    Serial.println(responseCode);
    if (responseCode < 0) forkAddressKnown = false;
  }
  http.end();
}

void processForkNowPacket(const uint8_t* data, int length) {
  if (length != sizeof(ForkRackNowPacket)) return;

  ForkRackNowPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic == ESP_NOW_MAGIC &&
      packet.version == ESP_NOW_VERSION &&
      packet.type == ESP_NOW_STATUS_REQUEST) {
    forkRequestedStatus = true;
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
  processForkNowPacket(data, length);
}
#else
void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int length) {
  processForkNowPacket(data, length);
}
#endif

bool startRackEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed");
    return false;
  }
  esp_now_register_recv_cb(onEspNowReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESP_NOW_BROADCAST_ADDRESS, 6);
  peer.channel = ESP_NOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(ESP_NOW_BROADCAST_ADDRESS) &&
      esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: ESP-NOW broadcast peer could not be added");
    return false;
  }
  Serial.println("ESP-NOW rack node ready");
  return true;
}

void sendRackStatusNow() {
  if (!espNowReady) return;

  ForkRackNowPacket packet = {};
  packet.magic = ESP_NOW_MAGIC;
  packet.version = ESP_NOW_VERSION;
  packet.type = ESP_NOW_RACK_STATUS;
  packet.sequence = ++espNowSequence;
  for (uint8_t i = 0; i < 4; i++) {
    packet.slots[i] = slotOccupied[i] ? 1 : 0;
  }
  packet.uptimeMS = millis();
  const esp_err_t result = esp_now_send(
      ESP_NOW_BROADCAST_ADDRESS,
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));
  if (result != ESP_OK) {
    Serial.print("Rack ESP-NOW send failed, code: ");
    Serial.println(static_cast<int>(result));
  }
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 Rack Occupancy Monitor");
  Serial.println("LOW = occupied, HIGH = empty");

  // IR modules provide their own digital output,
  // so ordinary INPUT mode is appropriate.
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  updateSensors();
  printSlotStatus();

  startRackSetupAccessPoint();
  espNowReady = startRackEspNow();

  server.on("/", HTTP_GET, handleMainPage);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/wifi", HTTP_GET, handleWiFiSetup);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("Rack diagnostic web server started.");
  Serial.println("=================================");
  Serial.println("Rack uses ESP-NOW only; no router or fork Wi-Fi login is required.");
  Serial.println("Optional rack diagnostic page: http://192.168.11.1");
  Serial.println("=================================");
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  server.handleClient();

  // Update sensors every 100 milliseconds
  if (
    millis() - previousSensorUpdate >=
    SENSOR_UPDATE_INTERVAL_MS
  ) {
    previousSensorUpdate = millis();
    updateSensors();
  }

  // Print status every second
  if (
    millis() - previousSerialPrint >=
    SERIAL_PRINT_INTERVAL_MS
  ) {
    previousSerialPrint = millis();
    printSlotStatus();
  }

  if (
    millis() - previousRackReport >=
    RACK_REPORT_INTERVAL_MS
  ) {
    previousRackReport = millis();
    sendRackStatusNow();
  }

  if (forkRequestedStatus) {
    forkRequestedStatus = false;
    sendRackStatusNow();
  }

}
