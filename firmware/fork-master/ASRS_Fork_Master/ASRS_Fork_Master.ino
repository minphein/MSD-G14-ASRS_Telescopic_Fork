#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#include <ASRSCommunication.h>
#include <ASRS_Master.h>
#include <ASRS_Comm_ESPNow.h>

#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// =====================================================

// HARDWARE & PIN DEFINITIONS

// =====================================================

#define STEP_PIN 14

#define DIR_PIN 27

#define EN_PIN 26

#define I2C_SDA_PIN 21

#define I2C_SCL_PIN 22

// Connect the LM393 module's analog output (AO) to this ADC1 pin.
// GPIO 34 is input-only and can be read while the ESP32 Wi-Fi is active.
#define LOAD_SENSOR_PIN 34


const int FULL_STEPS_PER_REV = 200;

const int MICROSTEPS = 4;

const float PINION_DIAMETER_MM = 25.0f;



// Change this in code if the controller starts with the second stage already
// sitting at a known position.
const float INITIAL_SECOND_STAGE_POS_MM = 0.0f;

// Third/top section movement ratio.
// Example: 150 mm of second-stage motor/rack travel moves the third/top section
// to the 300 mm coordinate, so the top section travel ratio is 2.0.
const float TOP_MM_PER_MOTOR_MM = 2.0f;



// Speed setting (Smaller = Faster)

const unsigned int STEP_DELAY_US = 800;

const bool INVERT_DIRECTION = false;

// Sensor homing settings. The motor moves in the positive direction until the
// VL53L0X reads this distance, then retracts by 320 mm of third-section travel.
const uint16_t HOMING_SENSOR_DISTANCE_MM = 30;

const float HOMING_RETRACT_TOP_MM = 320.0f;

const float MAX_HOMING_SECOND_STAGE_TRAVEL_MM = 1000.0f;

const int HOMING_STEPS_PER_SENSOR_READ = 10;

// The observed analog values are about 600+ with no load and about 22 with a
// load. Separate ON/OFF thresholds add hysteresis and prevent display chatter.
const int LOAD_DETECTED_THRESHOLD = 250;
const int LOAD_RELEASED_THRESHOLD = 350;
const uint8_t LOAD_SENSOR_SAMPLES = 8;



// =====================================================

// MATH CALCULATION

// =====================================================

const float DISTANCE_PER_REV_MM = PI * PINION_DIAMETER_MM; // ~78.54 mm



// Steps needed per 1 mm of MOTOR movement

const float STEPS_PER_MM_MOTOR = (FULL_STEPS_PER_REV * MICROSTEPS) / DISTANCE_PER_REV_MM;



// Steps needed per 1 mm of SECOND-STAGE motor/rack movement

const float STEPS_PER_MM_SECOND_STAGE = STEPS_PER_MM_MOTOR;



// Track state in mm

float currentSecondStagePosMM = INITIAL_SECOND_STAGE_POS_MM;

bool isMoving = false;

bool isHoming = false;

bool sensorReady = false;

bool homeConfigured = false;

uint16_t lastSensorDistanceMM = 0;

String lastHomingMessage = "Not homed";

int loadSensorValue = 0;

bool loadDetected = false;

bool rackSlotOccupied[4] = {false, false, false, false};

unsigned long lastRackUpdateMS = 0;

bool rackHasReported = false;

const unsigned long RACK_OFFLINE_TIMEOUT_MS = 5000;

const uint32_t ESP_NOW_MAGIC = 0x46524B31; // "FRK1"
const uint8_t ESP_NOW_VERSION = 1;
const uint8_t ESP_NOW_RACK_STATUS = 1;
const uint8_t ESP_NOW_STATUS_REQUEST = 2;
const unsigned long ESP_NOW_REQUEST_INTERVAL_MS = 2000;

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
uint16_t espNowSequence = 0;
unsigned long previousEspNowRequestMS = 0;

// The ASRS library owns the single ESP-NOW receive callback. Rack packets are
// separated by the raw receive handler below; all other packets continue into
// the ASRS protocol decoder.
ASRS_Comm_ESPNow asrsCommunication;
ASRS_ESPNow_MasterSession asrsSession(asrsCommunication);
ASRS_Master asrsMaster(asrsCommunication);

ASRS_Coordinates towerCoordinates = {0, 0};
ASRS_LimitSwitches towerLimits = {false, false, false, false};
bool towerCoordinatesValid = false;
bool towerLimitsValid = false;
bool towerSessionStarted = false;
String towerOperation = "Idle";
String towerMessage = "Waiting for ASRS tower";
bool towerOperationCompletedEvent = false;
bool towerOperationSucceeded = false;
unsigned long previousTowerTelemetryMS = 0;
unsigned long previousTowerSessionUpdateMS = 0;
const unsigned long TOWER_TELEMETRY_INTERVAL_MS = 2500;
const unsigned long TOWER_PAIRING_RETRY_INTERVAL_MS = 500;
// Pair after the channel-1 access point has started. A zero initial timeout
// initializes the transport without blocking setup on the radio's old channel.
const uint32_t TOWER_INITIAL_PAIRING_TIMEOUT_MS = 0;
const uint32_t TOWER_COMMAND_TIMEOUT_MS = 1500;
const int32_t AUTO_Z_STEP_MM = 2;
const int32_t AUTO_Z_MAX_SEARCH_MM = 100;
const unsigned long AUTO_Z_STEP_PAUSE_MS = 150;

// XYZ assignment workflow from the tower integration documentation:
// X/Z are absolute tower coordinates in millimetres and Y is the fork
// extension coordinate. After Y extends, Z probes upward/downward in small
// steps and the fork pressure sensor identifies pickup and release.
enum AutoTransferState : uint8_t {
    AUTO_IDLE,
    AUTO_MOVE_TO_SOURCE,
    AUTO_WAIT_SOURCE_TOWER,
    AUTO_EXTEND_AT_SOURCE,
    AUTO_RAISE_FOR_LOAD,
    AUTO_WAIT_RAISE_STEP,
    AUTO_RETRACT_FROM_SOURCE,
    AUTO_MOVE_TO_DESTINATION,
    AUTO_WAIT_DESTINATION_TOWER,
    AUTO_EXTEND_AT_DESTINATION,
    AUTO_LOWER_FOR_UNLOAD,
    AUTO_WAIT_LOWER_STEP,
    AUTO_RETRACT_FROM_DESTINATION,
    AUTO_COMPLETE,
    AUTO_ERROR
};

AutoTransferState autoTransferState = AUTO_IDLE;
int32_t autoSourceX = 0;
float autoSourceY = 0.0f;
int32_t autoSourceZ = 0;
int32_t autoDestinationX = 0;
float autoDestinationY = 0.0f;
int32_t autoDestinationZ = 0;
int32_t autoWorkingZ = 0;
unsigned long autoPreviousZStepMS = 0;
String autoTransferMessage = "Idle";
bool autoAbortRequested = false;

struct SlotCoordinate {
    int32_t x;
    float y;
    int32_t z;
    bool configured;
};

SlotCoordinate slotCoordinates[4] = {};
uint8_t autoSourceSlot = 0;       // 1-4 for slot transfers; 0 for free XYZ.
uint8_t autoDestinationSlot = 0;

Adafruit_VL53L0X distanceSensor = Adafruit_VL53L0X();



void updateLoadSensor() {

    uint32_t total = 0;

    for (uint8_t i = 0; i < LOAD_SENSOR_SAMPLES; i++) {

        total += analogRead(LOAD_SENSOR_PIN);

    }

    loadSensorValue = total / LOAD_SENSOR_SAMPLES;

    if (!loadDetected && loadSensorValue <= LOAD_DETECTED_THRESHOLD) {

        loadDetected = true;

    } else if (loadDetected && loadSensorValue >= LOAD_RELEASED_THRESHOLD) {

        loadDetected = false;

    }

}

bool rackIsConnected() {
    return rackHasReported &&
           millis() - lastRackUpdateMS <= RACK_OFFLINE_TIMEOUT_MS;
}

void loadSavedSlotCoordinates() {
    Preferences preferences;
    if (!preferences.begin("fork_slots", true)) {
        Serial.println("WARNING: Could not open saved slot coordinates");
        return;
    }

    char key[8];
    for (uint8_t i = 0; i < 4; i++) {
        snprintf(key, sizeof(key), "s%ux", i + 1);
        slotCoordinates[i].x = preferences.getInt(key, 0);
        snprintf(key, sizeof(key), "s%uy", i + 1);
        slotCoordinates[i].y = preferences.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "s%uz", i + 1);
        slotCoordinates[i].z = preferences.getInt(key, 0);
        snprintf(key, sizeof(key), "s%uok", i + 1);
        slotCoordinates[i].configured = preferences.getBool(key, false);
    }
    preferences.end();
}

bool saveSlotCoordinate(uint8_t slotIndex, int32_t x, float y, int32_t z) {
    if (slotIndex >= 4) return false;

    Preferences preferences;
    if (!preferences.begin("fork_slots", false)) return false;

    char key[8];
    snprintf(key, sizeof(key), "s%ux", slotIndex + 1);
    preferences.putInt(key, x);
    snprintf(key, sizeof(key), "s%uy", slotIndex + 1);
    preferences.putFloat(key, y);
    snprintf(key, sizeof(key), "s%uz", slotIndex + 1);
    preferences.putInt(key, z);
    snprintf(key, sizeof(key), "s%uok", slotIndex + 1);
    preferences.putBool(key, true);
    preferences.end();

    slotCoordinates[slotIndex] = {x, y, z, true};
    return true;
}

// =====================================================

// WEB SERVER SETUP

// =====================================================

const char* SETUP_AP_SSID = "Fork-WiFi-Setup";

const char* SETUP_AP_PASSWORD = "forksetup";

IPAddress forkAPIP(192, 168, 10, 1);

IPAddress forkAPGateway(192, 168, 10, 1);

IPAddress forkAPSubnet(255, 255, 255, 0);

const unsigned long AP_HEALTH_CHECK_INTERVAL_MS = 5000;

unsigned long previousAPHealthCheckMS = 0;

const uint8_t FORK_WIFI_CHANNEL = 1;

const uint8_t FORK_MAX_CLIENTS = 4;

bool mdnsStarted = false;



// Web interface operates on standard HTTP Port 80

WebServer server(80);

bool processAuxiliaryEspNowPacket(const uint8_t* senderMac, const uint8_t* data, int length) {
    (void)senderMac;
    if (length != sizeof(ForkRackNowPacket)) return false;

    ForkRackNowPacket packet;
    memcpy(&packet, data, sizeof(packet));
    if (packet.magic != ESP_NOW_MAGIC ||
        packet.version != ESP_NOW_VERSION ||
        packet.type != ESP_NOW_RACK_STATUS) return false;

    for (uint8_t i = 0; i < 4; i++) {
        rackSlotOccupied[i] = packet.slots[i] != 0;
    }
    lastRackUpdateMS = millis();
    rackHasReported = true;
    return true;
}

bool startIntegratedEspNow() {
    // Select the shared radio channel before the ASRS master starts pairing.
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (esp_wifi_set_channel(FORK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println("ERROR: Could not select ESP-NOW channel 1");
        return false;
    }

    ASRS_Comm_ESPNow::setRawReceiveHandler(processAuxiliaryEspNowPacket);
    if (!asrsSession.begin(FORK_WIFI_CHANNEL,
                           TOWER_INITIAL_PAIRING_TIMEOUT_MS,
                           &Serial)) {
        Serial.print("ERROR: ASRS ESP-NOW initialization failed: ");
        Serial.println(asrsErrorName(asrsSession.lastError()));
        return false;
    }

    // Short retry windows keep the fork web page responsive if the tower is off.
    asrsSession.setRepairingTimeout(50);
    asrsSession.setHeartbeatInterval(2000);
    asrsSession.setLinkTimeout(7000);
    towerSessionStarted = true;
    towerMessage = asrsSession.connected() ? "ASRS tower connected" : "Pairing with ASRS tower";
    Serial.println("Integrated rack + ASRS ESP-NOW coordinator ready");
    return true;
}

void requestRackStatusNow() {
    if (!espNowReady) return;
    ForkRackNowPacket packet = {};
    packet.magic = ESP_NOW_MAGIC;
    packet.version = ESP_NOW_VERSION;
    packet.type = ESP_NOW_STATUS_REQUEST;
    packet.sequence = ++espNowSequence;
    packet.uptimeMS = millis();
    esp_now_send(ESP_NOW_BROADCAST_ADDRESS,
                 reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}



bool startForkSetupAccessPoint() {

    // Keep station mode active because the ASRS ESP-NOW transport uses it.
    WiFi.mode(WIFI_AP_STA);

    WiFi.setSleep(false);

    if (!WiFi.softAPConfig(forkAPIP, forkAPGateway, forkAPSubnet)) {

        Serial.println("ERROR: Fork AP IP configuration failed");

        return false;

    }

    // hidden=false keeps the SSID visible; max clients allows the rack plus
    // phones/laptops to remain connected simultaneously.
    if (!WiFi.softAP(SETUP_AP_SSID, SETUP_AP_PASSWORD, FORK_WIFI_CHANNEL, false, FORK_MAX_CLIENTS)) {

        Serial.println("ERROR: Fork Wi-Fi access point failed to start");

        return false;

    }

    Serial.print("Fork setup AP ready at http://");

    Serial.println(WiFi.softAPIP());

    return true;

}



void startForkMDNS() {

    // mDNS is served directly to clients connected to the fork access point.
    if (!mdnsStarted && (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA)) {

        mdnsStarted = MDNS.begin("fork-control");

        if (mdnsStarted) {

            MDNS.addService("http", "tcp", 80);

            Serial.println("Fork available at http://fork-control.local");

        }

    }

}



void serviceNetworkDuringMotion() {

    server.handleClient();

    yield();

}



// =====================================================

// MOTOR CONTROL

// =====================================================

void pulseMotorStep() {

    digitalWrite(STEP_PIN, HIGH);

    delayMicroseconds(STEP_DELAY_US);

    digitalWrite(STEP_PIN, LOW);

    delayMicroseconds(STEP_DELAY_US);

}



void moveMotorSteps(long steps, bool direction) {

    bool actualDirection = direction ^ INVERT_DIRECTION;

    digitalWrite(DIR_PIN, actualDirection ? HIGH : LOW);

    delayMicroseconds(100); // DIR settle time



    for (long i = 0; i < steps; i++) {


        pulseMotorStep();



        // Service Wi-Fi about every 32 ms at the configured step speed.

        if (i % 20 == 0) serviceNetworkDuringMotion();

    }

}



bool readHomingDistance(uint16_t &distanceMM) {

    VL53L0X_RangingMeasurementData_t measurement;

    distanceSensor.rangingTest(&measurement, false);

    if (measurement.RangeStatus == 4) {

        return false;

    }

    distanceMM = measurement.RangeMilliMeter;

    lastSensorDistanceMM = distanceMM;

    return true;

}



bool runSensorHoming() {

    if (isMoving || !sensorReady) {

        lastHomingMessage = sensorReady ? "Motor is busy" : "VL53L0X not detected";

        return false;

    }

    isMoving = true;

    isHoming = true;

    homeConfigured = false;

    lastHomingMessage = "Searching for home sensor";

    digitalWrite(EN_PIN, LOW);



    bool actualDirection = true ^ INVERT_DIRECTION;

    digitalWrite(DIR_PIN, actualDirection ? HIGH : LOW);

    delayMicroseconds(100);



    const long maxHomingSteps = lroundf(

        MAX_HOMING_SECOND_STAGE_TRAVEL_MM * STEPS_PER_MM_SECOND_STAGE

    );

    long stepsMoved = 0;

    bool sensorTriggered = false;



    while (stepsMoved < maxHomingSteps) {

        uint16_t distanceMM;

        if (readHomingDistance(distanceMM) && distanceMM <= HOMING_SENSOR_DISTANCE_MM) {

            sensorTriggered = true;

            break;

        }



        for (int i = 0; i < HOMING_STEPS_PER_SENSOR_READ && stepsMoved < maxHomingSteps; i++) {

            pulseMotorStep();

            stepsMoved++;

        }

        serviceNetworkDuringMotion();

    }



    if (!sensorTriggered) {

        currentSecondStagePosMM += stepsMoved / STEPS_PER_MM_SECOND_STAGE;

        lastHomingMessage = "Home failed: sensor threshold not reached";

        homeConfigured = false;

        isHoming = false;

        isMoving = false;

        return false;

    }



    // The sensor point is 320 mm of third-section travel away from working zero.
    // Convert that top-section distance to the 160 mm second-stage retraction.
    const float retractSecondStageMM = HOMING_RETRACT_TOP_MM / TOP_MM_PER_MOTOR_MM;

    currentSecondStagePosMM = retractSecondStageMM;

    long retractSteps = lroundf(retractSecondStageMM * STEPS_PER_MM_SECOND_STAGE);

    moveMotorSteps(retractSteps, false);

    currentSecondStagePosMM = 0.0f;



    homeConfigured = true;

    lastHomingMessage = "Home position is configured";

    isHoming = false;

    isMoving = false;

    return true;

}



float getCurrentTopPosMM() {

    return currentSecondStagePosMM * TOP_MM_PER_MOTOR_MM;

}



void moveToSecondStagePosition(float targetSecondStageMM) {

    if (isMoving) return;

    isMoving = true;



    // Calculate distance the second stage motor/rack needs to travel

    float deltaSecondStageMM = targetSecondStageMM - currentSecondStagePosMM;



    // Calculate required steps

    long stepsToMove = lroundf(fabs(deltaSecondStageMM) * STEPS_PER_MM_SECOND_STAGE);



    if (stepsToMove > 0) {

        // Positive move = true direction, Negative move = false direction

        bool direction = (deltaSecondStageMM > 0);

        

        // Digital drive execution

        digitalWrite(EN_PIN, LOW); // Enable motor

        moveMotorSteps(stepsToMove, direction);

        

        // Update current position tracker

        currentSecondStagePosMM = targetSecondStageMM;

    }



    isMoving = false;

}


void moveToTopPosition(float targetTopMM) {

    // Web/API coordinates describe the third (top) section. Convert the
    // requested coordinate to the corresponding second-stage motor position.
    moveToSecondStagePosition(targetTopMM / TOP_MM_PER_MOTOR_MM);

}



// =====================================================

// WEB INTERFACE (HTML + JS)

// =====================================================

const char HTML_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

    <meta name="viewport" content="width=device-width, initial-scale=1">

    <title>MSD G14 Fork Control</title>

    <style>

        body { font-family: Arial, sans-serif; text-align: center; margin-top: 30px; background-color: #f4f4f9; color: #333; }

        .card { background: white; max-width: 400px; margin: auto; padding: 20px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); }

        h2 { margin-bottom: 5px; }

        .pos-display { font-size: 2rem; font-weight: bold; color: #007bff; margin: 15px 0; }

        .sub-text { font-size: 0.9rem; color: #666; margin-bottom: 20px; }

        input[type=number] { width: 70%; padding: 10px; font-size: 1.2rem; border: 1px solid #ccc; border-radius: 6px; text-align: center; }

        button { width: 80%; padding: 12px; margin: 8px 0; font-size: 1rem; border: none; border-radius: 6px; cursor: pointer; transition: 0.2s; }

        .btn-move { background-color: #007bff; color: white; font-weight: bold; }

        .btn-move:hover { background-color: #0056b3; }

        .btn-preset { background-color: #e2e8f0; color: #333; width: 24%; display: inline-block; }

        .btn-preset:hover { background-color: #cbd5e1; }

        .btn-zero { background-color: #dc3545; color: white; width: 80%; }

        .status { font-style: italic; color: #888; margin-top: 15px; }

        .home-state { margin: 12px 0; padding: 10px; border-radius: 6px; background-color: #fff3cd; color: #856404; font-weight: bold; }

        .home-state.configured { background-color: #d4edda; color: #155724; }

        .load-state { margin: 12px 0; padding: 12px; border-radius: 6px; background-color: #e2e8f0; color: #475569; font-weight: bold; }

        .load-state.detected { background-color: #d4edda; color: #155724; }

        .rack-panel { margin-top: 18px; padding: 12px; border-radius: 10px; background: #333; }

        .rack-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }

        .rack-slot { padding: 14px 5px; border-radius: 7px; color: white; background: #757575; font-weight: bold; }

        .rack-slot.occupied { background: #c62828; }

        .rack-slot.empty { background: #2e7d32; }

        .rack-connection { margin: 10px 0; font-weight: bold; color: #dc3545; }

        .rack-connection.online { color: #155724; }

        .tower-panel { margin-top: 20px; padding: 14px; border-radius: 10px; background: #eef6ff; }

        .tower-input { width: 42% !important; margin: 4px; }

        .tower-state { margin: 10px 0; font-weight: bold; color: #b91c1c; }

        .tower-state.online { color: #166534; }

        .xyz-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }

        .xyz-grid input { width: 100% !important; padding: 8px; font-size: 1rem; }

        .btn-abort { background-color: #b91c1c; color: white; }

        .slot-coordinate-row { display: grid; grid-template-columns: 42px repeat(3, 1fr); gap: 5px; margin: 7px 0; align-items: center; }

        .slot-coordinate-row input { width: 100% !important; padding: 7px; font-size: 0.9rem; }

        select { padding: 9px; margin: 5px; font-size: 1rem; }

    </style>

</head>

<body>

    <div class="card">

        <h2>MSD G14 Fork Control</h2>

        <div class="sub-text">Second Stage Target Control</div>

        <div class="sub-text">Fork webserver: <span id="forkWiFi">CHECKING</span> | <a href="/wifi">Wi-Fi setup</a></div>



        <div>Third Section Current Position:</div>

        <div class="pos-display"><span id="currPos">0.0</span> mm</div>



        <input type="number" id="targetInput" placeholder="Enter third section target (mm)" step="1" value="0"><br><br>

        <button class="btn-move" onclick="sendMove()">MOVE TO POSITION</button>



        <div style="margin-top: 15px;">

            <button class="btn-preset" onclick="setAndMove(-300)">-300</button>

            <button class="btn-preset" onclick="setAndMove(-150)">-150</button>

            <button class="btn-preset" onclick="setAndMove(150)">+150</button>

            <button class="btn-preset" onclick="setAndMove(300)">+300</button>

        </div>



        <hr style="margin: 20px 0; border: 0; border-top: 1px solid #eee;">

        <button class="btn-move" onclick="goHome()">HOME TO ZERO</button>

        <button class="btn-move" id="homingButton" onclick="sensorHome()">START HOMING</button>

        <div class="home-state" id="homeState">Home position is not configured</div>

        <div class="load-state" id="loadState">LOAD NOT DETECTED</div>

        <div class="sub-text">Pressure sensor: <span id="loadValue">--</span></div>

        <div class="rack-connection" id="rackConnection">RACK OFFLINE</div>

        <div class="rack-panel">

            <div class="rack-grid">

                <div class="rack-slot" id="rackSlot1">SLOT 1<br>UNKNOWN</div>

                <div class="rack-slot" id="rackSlot2">SLOT 2<br>UNKNOWN</div>

                <div class="rack-slot" id="rackSlot3">SLOT 3<br>UNKNOWN</div>

                <div class="rack-slot" id="rackSlot4">SLOT 4<br>UNKNOWN</div>

            </div>

        </div>

        <div class="sub-text" style="margin-top: 10px;">Occupied: <span id="rackOccupied">0</span> | Empty: <span id="rackEmpty">0</span></div>

        <div class="tower-panel">

            <h3>ASRS Tower X / Z</h3>

            <div class="tower-state" id="towerConnection">TOWER OFFLINE</div>

            <div>Current: X <span id="towerX">--</span>, Z <span id="towerZ">--</span></div>

            <input class="tower-input" type="number" id="towerTargetX" placeholder="Target X" step="1" value="0">

            <input class="tower-input" type="number" id="towerTargetZ" placeholder="Target Z" step="1" value="0">

            <button class="btn-move" onclick="moveTower()">MOVE X AND Z</button>

            <label><input type="checkbox" id="homeX" checked> Home X</label>&nbsp;&nbsp;

            <label><input type="checkbox" id="homeZ" checked> Home Z</label>

            <button class="btn-zero" onclick="homeTower()">HOME SELECTED AXES</button>

            <div class="sub-text">Operation: <span id="towerOperation">Idle</span></div>

            <div class="sub-text" id="towerMessage">Waiting for ASRS tower</div>

        </div>

        <div class="tower-panel">

            <h3>Automatic XYZ Transfer</h3>

            <div class="sub-text">Absolute coordinates in mm. Y is fork extension (-300 to +300).</div>

            <div class="sub-text">After Y extends, Z rises for pickup and lowers for placement until the pressure sensor changes.</div>

            <b>Pickup coordinate</b>

            <div class="xyz-grid">

                <input type="number" id="sourceX" placeholder="X" value="0">

                <input type="number" id="sourceY" placeholder="Y" value="300">

                <input type="number" id="sourceZ" placeholder="Z" value="0">

            </div>

            <br><b>Destination coordinate</b>

            <div class="xyz-grid">

                <input type="number" id="destinationX" placeholder="X" value="1600">

                <input type="number" id="destinationY" placeholder="Y" value="300">

                <input type="number" id="destinationZ" placeholder="Z" value="740">

            </div>

            <button class="btn-move" onclick="startTransfer()">START PICK AND PLACE</button>

            <button class="btn-abort" onclick="abortTransfer()">ABORT SEQUENCE</button>

            <div class="tower-state" id="autoState">IDLE</div>

            <div class="sub-text" id="autoMessage">Home the fork before starting.</div>

        </div>

        <div class="tower-panel">

            <h3>Saved Rack Slots</h3>

            <div class="sub-text">Enter each slot's absolute X/Y/Z coordinate in mm, then save it to ESP32 flash.</div>

            <div class="slot-coordinate-row"><b>S1</b><input id="s1x" placeholder="X"><input id="s1y" placeholder="Y"><input id="s1z" placeholder="Z"></div>

            <button class="btn-preset" onclick="saveSlot(1)">SAVE 1</button>

            <div class="slot-coordinate-row"><b>S2</b><input id="s2x" placeholder="X"><input id="s2y" placeholder="Y"><input id="s2z" placeholder="Z"></div>

            <button class="btn-preset" onclick="saveSlot(2)">SAVE 2</button>

            <div class="slot-coordinate-row"><b>S3</b><input id="s3x" placeholder="X"><input id="s3y" placeholder="Y"><input id="s3z" placeholder="Z"></div>

            <button class="btn-preset" onclick="saveSlot(3)">SAVE 3</button>

            <div class="slot-coordinate-row"><b>S4</b><input id="s4x" placeholder="X"><input id="s4y" placeholder="Y"><input id="s4z" placeholder="Z"></div>

            <button class="btn-preset" onclick="saveSlot(4)">SAVE 4</button>

            <hr>

            <label>Pick slot</label>

            <select id="pickSlot"><option>1</option><option>2</option><option>3</option><option>4</option></select>

            <label>Place slot</label>

            <select id="placeSlot"><option>1</option><option>2</option><option>3</option><option>4</option></select>

            <button class="btn-move" onclick="startSlotTransfer()">TRANSFER BETWEEN SLOTS</button>

            <div class="sub-text" id="slotSaveMessage">Configure all slots before use.</div>

        </div>



        <div class="status" id="statusText">Ready</div>

    </div>



    <script>

        let slotCoordinatesLoaded = false;

        function updateStatus() {

            return fetch('/status')

                .then(r => r.json())

                .then(data => {

                    document.getElementById('currPos').innerText = data.topPos.toFixed(1);

                    document.getElementById('forkWiFi').innerText = data.apIP;

                    const homeState = document.getElementById('homeState');

                    const loadState = document.getElementById('loadState');

                    document.getElementById('loadValue').innerText = data.loadSensorValue;

                    if (data.loadDetected) {

                        loadState.innerText = 'LOAD DETECTED';

                        loadState.classList.add('detected');

                    } else {

                        loadState.innerText = 'LOAD NOT DETECTED';

                        loadState.classList.remove('detected');

                    }

                    updateRackDisplay(data);

                    updateTowerDisplay(data);

                    document.getElementById('autoState').innerText = data.autoTransferActive ? 'TRANSFER RUNNING' : 'IDLE';

                    document.getElementById('autoState').classList.toggle('online', data.autoTransferActive);

                    document.getElementById('autoMessage').innerText = data.autoTransferMessage;

                    if (!slotCoordinatesLoaded) {

                        for (let i = 1; i <= 4; i++) {

                            const saved = data['slot' + i + 'Configured'];

                            document.getElementById('s' + i + 'x').value = saved ? data['slot' + i + 'X'] : '';

                            document.getElementById('s' + i + 'y').value = saved ? data['slot' + i + 'Y'] : '';

                            document.getElementById('s' + i + 'z').value = saved ? data['slot' + i + 'Z'] : '';

                        }

                        slotCoordinatesLoaded = true;

                    }

                    if (data.homing) {

                        homeState.innerText = 'Homing in progress...';

                        homeState.classList.remove('configured');

                    } else if (data.homeConfigured) {

                        homeState.innerText = 'Home position is configured';

                        homeState.classList.add('configured');

                    } else {

                        homeState.innerText = 'Home position is not configured';

                        homeState.classList.remove('configured');

                    }

                    if (data.moving) {

                        document.getElementById('statusText').innerText = 'Moving...';

                    } else {

                        document.getElementById('statusText').innerText = 'Ready';

                    }

                });

        }



        function updateRackDisplay(data) {

            const connection = document.getElementById('rackConnection');

            connection.innerText = data.rackConnected ? 'RACK CONNECTED (ESP-NOW)' : 'RACK OFFLINE';

            connection.classList.toggle('online', data.rackConnected);

            document.getElementById('rackOccupied').innerText = data.rackConnected ? data.rackOccupiedCount : 0;

            document.getElementById('rackEmpty').innerText = data.rackConnected ? data.rackEmptyCount : 0;

            for (let i = 1; i <= 4; i++) {

                const slot = document.getElementById('rackSlot' + i);

                slot.className = 'rack-slot';

                if (!data.rackConnected) {

                    slot.innerHTML = 'SLOT ' + i + '<br>UNKNOWN';

                } else if (data['rackSlot' + i]) {

                    slot.innerHTML = 'SLOT ' + i + '<br>OCCUPIED';

                    slot.classList.add('occupied');

                } else {

                    slot.innerHTML = 'SLOT ' + i + '<br>EMPTY';

                    slot.classList.add('empty');

                }

            }

        }

        function updateTowerDisplay(data) {

            const connection = document.getElementById('towerConnection');

            connection.innerText = data.towerConnected ? 'TOWER CONNECTED (ESP-NOW)' : 'TOWER OFFLINE';

            connection.classList.toggle('online', data.towerConnected);

            document.getElementById('towerX').innerText = data.towerCoordinatesValid ? data.towerX : '--';

            document.getElementById('towerZ').innerText = data.towerCoordinatesValid ? data.towerZ : '--';

            document.getElementById('towerOperation').innerText = data.towerOperation;

            document.getElementById('towerMessage').innerText = data.towerMessage;

        }

        function towerRequest(url) {

            document.getElementById('towerMessage').innerText = 'Sending command...';

            fetch(url)

                .then(async r => ({ ok: r.ok, message: await r.text() }))

                .then(result => {

                    document.getElementById('towerMessage').innerText = result.message;

                    return updateStatus();

                })

                .catch(() => document.getElementById('towerMessage').innerText = 'Tower request failed');

        }

        function moveTower() {

            const x = document.getElementById('towerTargetX').value;

            const z = document.getElementById('towerTargetZ').value;

            if (x === '' || z === '') return;

            towerRequest('/tower-move?x=' + encodeURIComponent(x) + '&z=' + encodeURIComponent(z));

        }

        function homeTower() {

            const x = document.getElementById('homeX').checked ? '1' : '0';

            const z = document.getElementById('homeZ').checked ? '1' : '0';

            towerRequest('/tower-home?x=' + x + '&z=' + z);

        }

        function startTransfer() {

            const ids = ['sourceX', 'sourceY', 'sourceZ', 'destinationX', 'destinationY', 'destinationZ'];

            const values = ids.map(id => document.getElementById(id).value);

            if (values.some(value => value === '')) return;

            const query = '?sx=' + encodeURIComponent(values[0]) +

                          '&sy=' + encodeURIComponent(values[1]) +

                          '&sz=' + encodeURIComponent(values[2]) +

                          '&dx=' + encodeURIComponent(values[3]) +

                          '&dy=' + encodeURIComponent(values[4]) +

                          '&dz=' + encodeURIComponent(values[5]);

            towerRequest('/auto-transfer' + query);

        }

        function abortTransfer() {

            towerRequest('/auto-abort');

        }

        function saveSlot(slot) {

            const x = document.getElementById('s' + slot + 'x').value;

            const y = document.getElementById('s' + slot + 'y').value;

            const z = document.getElementById('s' + slot + 'z').value;

            if (x === '' || y === '' || z === '') return;

            fetch('/slot-save?slot=' + slot + '&x=' + encodeURIComponent(x) +

                  '&y=' + encodeURIComponent(y) + '&z=' + encodeURIComponent(z))

                .then(async r => ({ ok: r.ok, message: await r.text() }))

                .then(result => {

                    document.getElementById('slotSaveMessage').innerText = result.message;

                    slotCoordinatesLoaded = false;

                    return updateStatus();

                });

        }

        function startSlotTransfer() {

            const source = document.getElementById('pickSlot').value;

            const destination = document.getElementById('placeSlot').value;

            towerRequest('/slot-transfer?source=' + source + '&destination=' + destination);

        }



        function sendMove() {

            let val = document.getElementById('targetInput').value;

            if (val === '') return;

            document.getElementById('statusText').innerText = 'Command sent...';

            fetch('/move?pos=' + val)

                .then(() => setTimeout(updateStatus, 500));

        }



        function setAndMove(val) {

            document.getElementById('targetInput').value = val;

            sendMove();

        }



        function goHome() {

            document.getElementById('targetInput').value = 0;

            sendMove();

        }



        function sensorHome() {

            const button = document.getElementById('homingButton');

            const homeState = document.getElementById('homeState');

            button.disabled = true;

            homeState.innerText = 'Homing in progress...';

            homeState.classList.remove('configured');

            document.getElementById('statusText').innerText = 'Moving toward sensor...';

            fetch('/sensorhome')

                .then(async r => ({ ok: r.ok, message: await r.text() }))

                .then(result => {

                    document.getElementById('statusText').innerText = result.message;

                    return updateStatus();

                })

                .catch(() => {

                    document.getElementById('statusText').innerText = 'Homing request failed';

                })

                .finally(() => {

                    button.disabled = false;

                });

        }



        setInterval(updateStatus, 1000);

        updateStatus();

    </script>

</body>

</html>

)rawliteral";



// =====================================================

// ROUTE HANDLERS

// =====================================================

void updateTowerOperation() {
    if (!asrsMaster.operationActive()) return;

    ASRS_OperationStatus status;
    if (!asrsMaster.readOperationStatus(status)) return;

    asrsSession.recordPeerActivity();
    if (status.status == ASRS_STATUS_BUSY) {
        towerMessage = towerOperation + " in progress";
    } else if (status.status == ASRS_STATUS_DONE) {
        towerMessage = towerOperation + " completed";
        towerOperation = "Idle";
        towerOperationSucceeded = true;
        towerOperationCompletedEvent = true;
        previousTowerTelemetryMS = 0;
    } else if (status.status == ASRS_STATUS_ERROR) {
        towerMessage = towerOperation + " failed: " + String(asrsErrorName(status.error));
        towerOperation = "Idle";
        towerOperationSucceeded = false;
        towerOperationCompletedEvent = true;
    }
}

bool startTowerMove(int32_t targetX, int32_t targetZ, const String &operationName) {
    if (!asrsSession.connected() || asrsMaster.operationActive()) return false;

    towerOperationCompletedEvent = false;
    towerOperationSucceeded = false;
    if (!asrsMaster.sendTravelCommand(targetX, targetZ, TOWER_COMMAND_TIMEOUT_MS)) {
        towerMessage = operationName + " rejected: " + String(asrsErrorName(asrsMaster.lastError()));
        return false;
    }

    asrsSession.recordPeerActivity();
    towerOperation = operationName;
    towerMessage = operationName + " accepted: X=" + String(targetX) + ", Z=" + String(targetZ);
    return true;
}

void failAutoTransfer(const String &message) {
    autoTransferMessage = message;
    autoTransferState = AUTO_ERROR;
}

void updateAutoTransfer() {
    if (autoTransferState == AUTO_IDLE) return;

    updateLoadSensor();

    if (autoAbortRequested && !asrsMaster.operationActive()) {
        autoAbortRequested = false;
        autoTransferMessage = "Transfer aborted; retracting fork";
        moveToTopPosition(0.0f);
        autoTransferState = AUTO_IDLE;
        return;
    }

    switch (autoTransferState) {
        case AUTO_MOVE_TO_SOURCE:
            if (autoSourceSlot != 0) {
                if (!rackIsConnected()) {
                    failAutoTransfer("Slot transfer stopped: rack is offline");
                    break;
                }
                if (!rackSlotOccupied[autoSourceSlot - 1]) {
                    failAutoTransfer("Slot transfer stopped: pickup slot is empty");
                    break;
                }
            }
            autoTransferMessage = "Moving tower to pickup X/Z";
            if (startTowerMove(autoSourceX, autoSourceZ, "Moving to pickup")) {
                autoTransferState = AUTO_WAIT_SOURCE_TOWER;
            } else {
                failAutoTransfer("Could not start pickup tower move: " + towerMessage);
            }
            break;

        case AUTO_WAIT_SOURCE_TOWER:
            if (towerOperationCompletedEvent) {
                towerOperationCompletedEvent = false;
                if (towerOperationSucceeded) {
                    autoWorkingZ = autoSourceZ;
                    autoTransferState = AUTO_EXTEND_AT_SOURCE;
                } else {
                    failAutoTransfer("Tower failed to reach pickup coordinate");
                }
            }
            break;

        case AUTO_EXTEND_AT_SOURCE:
            if (autoSourceSlot != 0 &&
                (!rackIsConnected() || !rackSlotOccupied[autoSourceSlot - 1])) {
                failAutoTransfer("Pickup cancelled: source slot became empty or rack went offline");
                break;
            }
            autoTransferMessage = "Extending fork to pickup Y=" + String(autoSourceY, 1) + " mm";
            moveToTopPosition(autoSourceY);
            autoTransferMessage = "Raising Z slowly until payload is detected";
            autoPreviousZStepMS = 0;
            autoTransferState = AUTO_RAISE_FOR_LOAD;
            break;

        case AUTO_RAISE_FOR_LOAD:
            if (loadDetected) {
                autoTransferMessage = "Payload detected; retracting fork";
                autoTransferState = AUTO_RETRACT_FROM_SOURCE;
                break;
            }
            if (autoWorkingZ - autoSourceZ >= AUTO_Z_MAX_SEARCH_MM) {
                failAutoTransfer("Pickup stopped: no load detected within Z search limit");
                break;
            }
            if (autoPreviousZStepMS != 0 &&
                millis() - autoPreviousZStepMS < AUTO_Z_STEP_PAUSE_MS) break;

            autoWorkingZ += AUTO_Z_STEP_MM;
            autoTransferMessage = "Pickup Z=" + String(autoWorkingZ) +
                                  " mm; pressure=" + String(loadSensorValue);
            if (startTowerMove(autoSourceX, autoWorkingZ, "Pickup Z step")) {
                autoPreviousZStepMS = millis();
                autoTransferState = AUTO_WAIT_RAISE_STEP;
            } else {
                failAutoTransfer("Could not raise Z during pickup: " + towerMessage);
            }
            break;

        case AUTO_WAIT_RAISE_STEP:
            if (towerOperationCompletedEvent) {
                towerOperationCompletedEvent = false;
                if (!towerOperationSucceeded) {
                    failAutoTransfer("Tower Z step failed during pickup");
                } else if (loadDetected) {
                    autoTransferMessage = "Payload detected; retracting fork";
                    autoTransferState = AUTO_RETRACT_FROM_SOURCE;
                } else {
                    autoPreviousZStepMS = millis();
                    autoTransferState = AUTO_RAISE_FOR_LOAD;
                }
            }
            break;

        case AUTO_RETRACT_FROM_SOURCE:
            moveToTopPosition(0.0f);
            autoTransferState = AUTO_MOVE_TO_DESTINATION;
            break;

        case AUTO_MOVE_TO_DESTINATION:
            if (autoDestinationSlot != 0) {
                if (!rackIsConnected()) {
                    failAutoTransfer("Slot transfer stopped: rack is offline");
                    break;
                }
                if (rackSlotOccupied[autoDestinationSlot - 1]) {
                    failAutoTransfer("Slot transfer stopped: destination slot is occupied");
                    break;
                }
            }
            autoTransferMessage = "Moving tower to destination X/Z";
            if (startTowerMove(autoDestinationX, autoDestinationZ, "Moving to destination")) {
                autoTransferState = AUTO_WAIT_DESTINATION_TOWER;
            } else {
                failAutoTransfer("Could not start destination tower move: " + towerMessage);
            }
            break;

        case AUTO_WAIT_DESTINATION_TOWER:
            if (towerOperationCompletedEvent) {
                towerOperationCompletedEvent = false;
                if (towerOperationSucceeded) {
                    autoWorkingZ = autoDestinationZ;
                    autoTransferState = AUTO_EXTEND_AT_DESTINATION;
                } else {
                    failAutoTransfer("Tower failed to reach destination coordinate");
                }
            }
            break;

        case AUTO_EXTEND_AT_DESTINATION:
            if (autoDestinationSlot != 0 &&
                (!rackIsConnected() || rackSlotOccupied[autoDestinationSlot - 1])) {
                failAutoTransfer("Placement cancelled: destination became occupied or rack went offline");
                break;
            }
            autoTransferMessage = "Extending fork to destination Y=" + String(autoDestinationY, 1) + " mm";
            moveToTopPosition(autoDestinationY);
            autoTransferMessage = "Lowering Z slowly until payload is released";
            autoPreviousZStepMS = 0;
            autoTransferState = AUTO_LOWER_FOR_UNLOAD;
            break;

        case AUTO_LOWER_FOR_UNLOAD:
            if (!loadDetected) {
                autoTransferMessage = "Payload removed; retracting fork";
                autoTransferState = AUTO_RETRACT_FROM_DESTINATION;
                break;
            }
            if (autoDestinationZ - autoWorkingZ >= AUTO_Z_MAX_SEARCH_MM) {
                failAutoTransfer("Placement stopped: load still detected at Z search limit");
                break;
            }
            if (autoPreviousZStepMS != 0 &&
                millis() - autoPreviousZStepMS < AUTO_Z_STEP_PAUSE_MS) break;

            autoWorkingZ -= AUTO_Z_STEP_MM;
            autoTransferMessage = "Placement Z=" + String(autoWorkingZ) +
                                  " mm; pressure=" + String(loadSensorValue);
            if (startTowerMove(autoDestinationX, autoWorkingZ, "Placement Z step")) {
                autoPreviousZStepMS = millis();
                autoTransferState = AUTO_WAIT_LOWER_STEP;
            } else {
                failAutoTransfer("Could not lower Z during placement: " + towerMessage);
            }
            break;

        case AUTO_WAIT_LOWER_STEP:
            if (towerOperationCompletedEvent) {
                towerOperationCompletedEvent = false;
                if (!towerOperationSucceeded) {
                    failAutoTransfer("Tower Z step failed during placement");
                } else if (!loadDetected) {
                    autoTransferMessage = "Payload released; retracting fork";
                    autoTransferState = AUTO_RETRACT_FROM_DESTINATION;
                } else {
                    autoPreviousZStepMS = millis();
                    autoTransferState = AUTO_LOWER_FOR_UNLOAD;
                }
            }
            break;

        case AUTO_RETRACT_FROM_DESTINATION:
            moveToTopPosition(0.0f);
            autoTransferState = AUTO_COMPLETE;
            break;

        case AUTO_COMPLETE:
            autoTransferMessage = "XYZ transfer completed";
            autoTransferState = AUTO_IDLE;
            break;

        case AUTO_ERROR:
            if (!asrsMaster.operationActive()) {
                moveToTopPosition(0.0f);
                autoTransferState = AUTO_IDLE;
            }
            break;

        default:
            break;
    }
}

void updateTowerTelemetry() {
    if (!asrsSession.connected() || asrsMaster.operationActive() ||
        autoTransferState != AUTO_IDLE) return;
    if (millis() - previousTowerTelemetryMS < TOWER_TELEMETRY_INTERVAL_MS) return;

    previousTowerTelemetryMS = millis();

    ASRS_Coordinates coordinates;
    if (asrsMaster.requestCoordinates(coordinates, TOWER_COMMAND_TIMEOUT_MS)) {
        towerCoordinates = coordinates;
        towerCoordinatesValid = true;
        asrsSession.recordPeerActivity();
    }

    ASRS_LimitSwitches limits;
    if (asrsMaster.requestLimitSwitches(limits, TOWER_COMMAND_TIMEOUT_MS)) {
        towerLimits = limits;
        towerLimitsValid = true;
        asrsSession.recordPeerActivity();
    }
}

void handleTowerMove() {
    if (!server.hasArg("x") || !server.hasArg("z")) {
        server.send(400, "text/plain", "Both X and Z targets are required");
        return;
    }
    if (!asrsSession.connected()) {
        server.send(503, "text/plain", "ASRS tower is not connected");
        return;
    }
    if (asrsMaster.operationActive()) {
        server.send(409, "text/plain", "ASRS tower is busy");
        return;
    }
    if (autoTransferState != AUTO_IDLE) {
        server.send(409, "text/plain", "Automatic XYZ transfer is active");
        return;
    }

    const int32_t targetX = static_cast<int32_t>(server.arg("x").toInt());
    const int32_t targetZ = static_cast<int32_t>(server.arg("z").toInt());
    if (!startTowerMove(targetX, targetZ, "Moving X/Z")) {
        server.send(500, "text/plain", towerMessage);
        return;
    }

    server.send(202, "text/plain", towerMessage);
}

void handleTowerHome() {
    const bool homeX = server.hasArg("x") && server.arg("x") == "1";
    const bool homeZ = server.hasArg("z") && server.arg("z") == "1";
    if (!homeX && !homeZ) {
        server.send(400, "text/plain", "Select X, Z, or both axes to home");
        return;
    }
    if (!asrsSession.connected()) {
        server.send(503, "text/plain", "ASRS tower is not connected");
        return;
    }
    if (asrsMaster.operationActive()) {
        server.send(409, "text/plain", "ASRS tower is busy");
        return;
    }
    if (autoTransferState != AUTO_IDLE) {
        server.send(409, "text/plain", "Automatic XYZ transfer is active");
        return;
    }

    if (!asrsMaster.sendHomingCommand(homeX, homeZ, TOWER_COMMAND_TIMEOUT_MS)) {
        towerMessage = "Homing rejected: " + String(asrsErrorName(asrsMaster.lastError()));
        server.send(500, "text/plain", towerMessage);
        return;
    }

    asrsSession.recordPeerActivity();
    towerOperation = "Homing";
    towerMessage = "Homing accepted: X=" + String(homeX ? "yes" : "no") +
                   ", Z=" + String(homeZ ? "yes" : "no");
    server.send(202, "text/plain", towerMessage);
}

void handleAutoTransfer() {
    const char* requiredArguments[] = {"sx", "sy", "sz", "dx", "dy", "dz"};
    for (const char* argument : requiredArguments) {
        if (!server.hasArg(argument)) {
            server.send(400, "text/plain", "Source and destination X/Y/Z values are required");
            return;
        }
    }

    if (!asrsSession.connected()) {
        server.send(503, "text/plain", "ASRS tower is not connected");
        return;
    }
    if (asrsMaster.operationActive() || autoTransferState != AUTO_IDLE) {
        server.send(409, "text/plain", "Tower or automatic transfer is busy");
        return;
    }
    if (!homeConfigured) {
        server.send(409, "text/plain", "Home the fork before starting an XYZ transfer");
        return;
    }

    updateLoadSensor();
    if (loadDetected) {
        server.send(409, "text/plain", "Remove the existing payload before starting");
        return;
    }

    const float sourceY = server.arg("sy").toFloat();
    const float destinationY = server.arg("dy").toFloat();
    if (sourceY < -300.0f || sourceY > 300.0f ||
        destinationY < -300.0f || destinationY > 300.0f) {
        server.send(400, "text/plain", "Y must be between -300 and +300 mm");
        return;
    }

    autoSourceX = static_cast<int32_t>(server.arg("sx").toInt());
    autoSourceY = sourceY;
    autoSourceZ = static_cast<int32_t>(server.arg("sz").toInt());
    autoDestinationX = static_cast<int32_t>(server.arg("dx").toInt());
    autoDestinationY = destinationY;
    autoDestinationZ = static_cast<int32_t>(server.arg("dz").toInt());
    autoSourceSlot = 0;
    autoDestinationSlot = 0;
    autoAbortRequested = false;
    towerOperationCompletedEvent = false;
    autoTransferMessage = "XYZ transfer accepted";
    autoTransferState = AUTO_MOVE_TO_SOURCE;
    server.send(202, "text/plain", autoTransferMessage);
}

void handleAutoAbort() {
    if (autoTransferState == AUTO_IDLE) {
        server.send(200, "text/plain", "No automatic transfer is active");
        return;
    }

    // The ASRS protocol has no motion-cancel command. If an X/Z move is active,
    // abort takes effect after the tower reports that move complete.
    autoAbortRequested = true;
    autoTransferMessage = asrsMaster.operationActive()
        ? "Abort requested; waiting for current tower move to finish"
        : "Abort requested";
    server.send(202, "text/plain", autoTransferMessage);
}

void handleSlotSave() {
    if (!server.hasArg("slot") || !server.hasArg("x") ||
        !server.hasArg("y") || !server.hasArg("z")) {
        server.send(400, "text/plain", "Slot number and X/Y/Z values are required");
        return;
    }

    const int slotNumber = server.arg("slot").toInt();
    const float y = server.arg("y").toFloat();
    if (slotNumber < 1 || slotNumber > 4) {
        server.send(400, "text/plain", "Slot number must be 1 to 4");
        return;
    }
    if (y < -300.0f || y > 300.0f) {
        server.send(400, "text/plain", "Slot Y must be between -300 and +300 mm");
        return;
    }

    const int32_t x = static_cast<int32_t>(server.arg("x").toInt());
    const int32_t z = static_cast<int32_t>(server.arg("z").toInt());
    if (!saveSlotCoordinate(slotNumber - 1, x, y, z)) {
        server.send(500, "text/plain", "Could not save slot coordinate to flash");
        return;
    }

    String message = "Slot " + String(slotNumber) + " saved: X=" + String(x) +
                     ", Y=" + String(y, 1) + ", Z=" + String(z);
    server.send(200, "text/plain", message);
}

void handleSlotTransfer() {
    if (!server.hasArg("source") || !server.hasArg("destination")) {
        server.send(400, "text/plain", "Pick and place slot numbers are required");
        return;
    }

    const int source = server.arg("source").toInt();
    const int destination = server.arg("destination").toInt();
    if (source < 1 || source > 4 || destination < 1 || destination > 4) {
        server.send(400, "text/plain", "Slot numbers must be 1 to 4");
        return;
    }
    if (source == destination) {
        server.send(400, "text/plain", "Pick and place slots must be different");
        return;
    }
    if (!slotCoordinates[source - 1].configured ||
        !slotCoordinates[destination - 1].configured) {
        server.send(409, "text/plain", "Save both slot coordinates before transferring");
        return;
    }
    if (!rackIsConnected()) {
        server.send(503, "text/plain", "Rack is offline; slot occupancy cannot be verified");
        return;
    }
    if (!rackSlotOccupied[source - 1]) {
        server.send(409, "text/plain", "Cannot pick: source slot is empty");
        return;
    }
    if (rackSlotOccupied[destination - 1]) {
        server.send(409, "text/plain", "Cannot place: destination slot is occupied");
        return;
    }
    if (!asrsSession.connected()) {
        server.send(503, "text/plain", "ASRS tower is not connected");
        return;
    }
    if (!homeConfigured) {
        server.send(409, "text/plain", "Home the fork before starting a slot transfer");
        return;
    }
    if (asrsMaster.operationActive() || autoTransferState != AUTO_IDLE) {
        server.send(409, "text/plain", "Tower or automatic transfer is busy");
        return;
    }

    updateLoadSensor();
    if (loadDetected) {
        server.send(409, "text/plain", "Remove the existing fork payload before starting");
        return;
    }

    const SlotCoordinate &sourceCoordinate = slotCoordinates[source - 1];
    const SlotCoordinate &destinationCoordinate = slotCoordinates[destination - 1];
    autoSourceX = sourceCoordinate.x;
    autoSourceY = sourceCoordinate.y;
    autoSourceZ = sourceCoordinate.z;
    autoDestinationX = destinationCoordinate.x;
    autoDestinationY = destinationCoordinate.y;
    autoDestinationZ = destinationCoordinate.z;
    autoSourceSlot = source;
    autoDestinationSlot = destination;
    autoAbortRequested = false;
    towerOperationCompletedEvent = false;
    autoTransferMessage = "Slot " + String(source) + " to slot " +
                          String(destination) + " transfer accepted";
    autoTransferState = AUTO_MOVE_TO_SOURCE;
    server.send(202, "text/plain", autoTransferMessage);
}

void handleRoot() {

    server.send(200, "text/html", HTML_PAGE);

}



void handleWiFiSetup() {
    String page = "<h2>Fork Network</h2><p>The fork is the ESP-NOW master and does not join a router.</p>";
    page += "<p>Connect directly to <b>Fork-WiFi-Setup</b> and open <b>http://192.168.10.1</b>.</p><p><a href='/'>Return to control page</a></p>";
    server.send(200, "text/html", page);
}



void handleMove() {

    if (autoTransferState != AUTO_IDLE) {

        server.send(409, "text/plain", "Automatic XYZ transfer is active");

        return;

    }

    if (server.hasArg("pos")) {

        float targetTopMM = server.arg("pos").toFloat();

        server.send(200, "text/plain", "OK");

        moveToTopPosition(targetTopMM);

    } else {

        server.send(400, "text/plain", "Bad Request");

    }

}



void handleStatus() {

    updateLoadSensor();

    bool rackConnected = rackIsConnected();

    uint8_t rackOccupiedCount = 0;

    for (uint8_t i = 0; i < 4; i++) {

        if (rackSlotOccupied[i]) rackOccupiedCount++;

    }

    String json = "{";

    json += "\"topPos\":" + String(getCurrentTopPosMM(), 2) + ",";

    json += "\"secondStagePos\":" + String(currentSecondStagePosMM, 2) + ",";

    json += "\"sensorDistance\":" + String(lastSensorDistanceMM) + ",";

    json += "\"loadSensorValue\":" + String(loadSensorValue) + ",";

    json += "\"loadDetected\":" + String(loadDetected ? "true" : "false") + ",";

    json += "\"wifiConnected\":" + String((WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) ? "true" : "false") + ",";

    json += "\"stationIP\":\"" + WiFi.localIP().toString() + "\",";

    json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\",";

    json += "\"espNowReady\":" + String(espNowReady ? "true" : "false") + ",";

    json += "\"rackConnected\":" + String(rackConnected ? "true" : "false") + ",";

    json += "\"rackSlot1\":" + String(rackSlotOccupied[0] ? "true" : "false") + ",";

    json += "\"rackSlot2\":" + String(rackSlotOccupied[1] ? "true" : "false") + ",";

    json += "\"rackSlot3\":" + String(rackSlotOccupied[2] ? "true" : "false") + ",";

    json += "\"rackSlot4\":" + String(rackSlotOccupied[3] ? "true" : "false") + ",";

    json += "\"rackOccupiedCount\":" + String(rackOccupiedCount) + ",";

    json += "\"rackEmptyCount\":" + String(4 - rackOccupiedCount) + ",";

    json += "\"towerConnected\":" + String(asrsSession.connected() ? "true" : "false") + ",";

    json += "\"towerBusy\":" + String(asrsMaster.operationActive() ? "true" : "false") + ",";

    json += "\"towerCoordinatesValid\":" + String(towerCoordinatesValid ? "true" : "false") + ",";

    json += "\"towerX\":" + String(towerCoordinates.x) + ",";

    json += "\"towerZ\":" + String(towerCoordinates.z) + ",";

    json += "\"towerLimitXMin\":" + String(towerLimits.xMinimum ? "true" : "false") + ",";

    json += "\"towerLimitXMax\":" + String(towerLimits.xMaximum ? "true" : "false") + ",";

    json += "\"towerLimitZMin\":" + String(towerLimits.zMinimum ? "true" : "false") + ",";

    json += "\"towerLimitZMax\":" + String(towerLimits.zMaximum ? "true" : "false") + ",";

    json += "\"towerOperation\":\"" + towerOperation + "\",";

    json += "\"towerMessage\":\"" + towerMessage + "\",";

    json += "\"autoTransferActive\":" + String(autoTransferState != AUTO_IDLE ? "true" : "false") + ",";

    json += "\"autoTransferMessage\":\"" + autoTransferMessage + "\",";

    for (uint8_t i = 0; i < 4; i++) {

        const String slotNumber = String(i + 1);

        json += "\"slot" + slotNumber + "Configured\":" +
                String(slotCoordinates[i].configured ? "true" : "false") + ",";

        json += "\"slot" + slotNumber + "X\":" + String(slotCoordinates[i].x) + ",";

        json += "\"slot" + slotNumber + "Y\":" + String(slotCoordinates[i].y, 1) + ",";

        json += "\"slot" + slotNumber + "Z\":" + String(slotCoordinates[i].z) + ",";

    }

    json += "\"homing\":" + String(isHoming ? "true" : "false") + ",";

    json += "\"homeConfigured\":" + String(homeConfigured ? "true" : "false") + ",";

    json += "\"moving\":" + String(isMoving ? "true" : "false");

    json += "}";

    server.send(200, "application/json", json);

}



void handleRackUpdate() {

    const char* names[4] = {"slot1", "slot2", "slot3", "slot4"};

    for (uint8_t i = 0; i < 4; i++) {

        if (!server.hasArg(names[i])) {

            server.send(400, "text/plain", "Missing slot value");

            return;

        }

    }

    for (uint8_t i = 0; i < 4; i++) {

        rackSlotOccupied[i] = server.arg(names[i]) == "1";

    }

    lastRackUpdateMS = millis();

    rackHasReported = true;

    Serial.print("Rack update received from ");

    Serial.println(server.client().remoteIP());

    server.send(200, "text/plain", "OK");

}



void handleSensorHome() {

    if (autoTransferState != AUTO_IDLE) {

        server.send(409, "text/plain", "Automatic XYZ transfer is active");

        return;

    }

    bool homed = runSensorHoming();

    server.send(homed ? 200 : 500, "text/plain", lastHomingMessage);

}



// =====================================================

// SETUP & LOOP

// =====================================================

void setup() {

    Serial.begin(115200);



    pinMode(EN_PIN, OUTPUT);

    pinMode(DIR_PIN, OUTPUT);

    pinMode(STEP_PIN, OUTPUT);

    pinMode(LOAD_SENSOR_PIN, INPUT);

    analogReadResolution(12);



    digitalWrite(STEP_PIN, LOW);

    digitalWrite(DIR_PIN, LOW);

    digitalWrite(EN_PIN, LOW); // Enable motor driver



    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    sensorReady = distanceSensor.begin();

    updateLoadSensor();

    loadSavedSlotCoordinates();



    // One channel and one receive callback are shared by the rack and tower.
    espNowReady = startIntegratedEspNow();

    // Start the control AP after ESP-NOW initialization, retaining STA mode.
    startForkSetupAccessPoint();

    startForkMDNS();

    IPAddress myIP = WiFi.softAPIP();



    Serial.println("\n--- Wi-Fi Control Ready ---");

    Serial.print("VL53L0X sensor: ");

    Serial.println(sensorReady ? "ready" : "not detected");

    Serial.println("For Wi-Fi setup connect to: Fork-WiFi-Setup");

    Serial.print("Setup page: http://");

    Serial.println(myIP);



    // Register Web Server URLs

    server.on("/", handleRoot);

    server.on("/wifi", HTTP_GET, handleWiFiSetup);

    server.on("/move", handleMove);

    server.on("/status", handleStatus);

    server.on("/sensorhome", handleSensorHome);

    server.on("/tower-move", HTTP_GET, handleTowerMove);

    server.on("/tower-home", HTTP_GET, handleTowerHome);

    server.on("/auto-transfer", HTTP_GET, handleAutoTransfer);

    server.on("/auto-abort", HTTP_GET, handleAutoAbort);

    server.on("/slot-save", HTTP_GET, handleSlotSave);

    server.on("/slot-transfer", HTTP_GET, handleSlotTransfer);

    server.on("/rack-update", HTTP_POST, handleRackUpdate);

    server.begin();

}



void loop() {

    server.handleClient();

    if (towerSessionStarted &&
        (asrsSession.connected() ||
         millis() - previousTowerSessionUpdateMS >= TOWER_PAIRING_RETRY_INTERVAL_MS)) {
        previousTowerSessionUpdateMS = millis();
        asrsSession.update();
    }

    if (towerSessionStarted) {
        updateTowerOperation();
        updateAutoTransfer();
        updateTowerTelemetry();

        if (!asrsSession.connected() && !asrsMaster.operationActive()) {
            towerMessage = "Pairing with ASRS tower on channel 1";
        }
    }

    if (millis() - previousEspNowRequestMS >= ESP_NOW_REQUEST_INTERVAL_MS) {
        previousEspNowRequestMS = millis();
        requestRackStatusNow();
    }

    if (millis() - previousAPHealthCheckMS >= AP_HEALTH_CHECK_INTERVAL_MS) {

        previousAPHealthCheckMS = millis();

        bool apInvalid = (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_AP) ||

                         WiFi.softAPIP() == IPAddress(0, 0, 0, 0) ||

                         WiFi.softAPSSID() != String(SETUP_AP_SSID);

        Serial.print("AP uptime(s): ");

        Serial.print(millis() / 1000);

        Serial.print(" | clients: ");

        Serial.print(WiFi.softAPgetStationNum());

        Serial.print(" | free heap: ");

        Serial.println(ESP.getFreeHeap());

        if (apInvalid) {

            Serial.println("Fork Wi-Fi stopped; attempting recovery...");

            WiFi.softAPdisconnect(true);

            startForkSetupAccessPoint();

        }

        startForkMDNS();

    }

} 
