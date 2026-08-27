#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <esp_now.h>
#include <ASRSCommunication.h>

#if !defined(ESP32)
#error "ASRS_Fork_Master requires an ESP32."
#endif

// ---------------- User configuration ----------------
constexpr char AP_SSID[] = "ASRS-Fork-Control";
constexpr char AP_PASSWORD[] = "asrscontrol";
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint32_t PAIR_TIMEOUT_MS = 0;
constexpr uint32_t REPAIR_ATTEMPT_TIMEOUT_MS = 50;
constexpr uint32_t TOWER_SESSION_UPDATE_MS = 1000;
constexpr uint32_t TOWER_OPERATION_TIMEOUT_MS = 120000;
constexpr uint32_t TOWER_WAIT_LOG_MS = 2000;
constexpr uint32_t TOWER_ACK_TIMEOUT_MS = 2000;
// The motion-enabled ASRS slave accepts absolute sensor-frame targets. The fork
// UI uses logical coordinates whose X=0/Z=0 origin is captured after homing.
constexpr bool TOWER_COMMANDS_ARE_RELATIVE = false;

constexpr uint8_t STEP_PIN = 14;
constexpr uint8_t DIR_PIN = 27;
constexpr uint8_t ENABLE_PIN = 26;
constexpr uint8_t LOAD_SENSOR_PIN = 34;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

constexpr float Y_MIN_MM = -300.0f;
constexpr float Y_MAX_MM = 300.0f;
constexpr float Y_RETRACTED_TOLERANCE_MM = 2.0f;
constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEPS = 4;
constexpr float PINION_DIAMETER_MM = 25.0f;
constexpr float TOP_MM_PER_MOTOR_MM = 2.0f;
constexpr float STEPS_PER_TOP_MM =
    ((FULL_STEPS_PER_REV * MICROSTEPS) / (PI * PINION_DIAMETER_MM)) /
    TOP_MM_PER_MOTOR_MM;
constexpr uint32_t STEP_HALF_PERIOD_US = 800;

constexpr uint16_t HOMING_SENSOR_DISTANCE_MM = 30;
constexpr float HOMING_RETRACT_TOP_MM = 320.0f;
constexpr float MAX_HOMING_SECOND_STAGE_TRAVEL_MM = 1000.0f;
const long MAX_HOMING_STEPS = lroundf(
    MAX_HOMING_SECOND_STAGE_TRAVEL_MM * STEPS_PER_TOP_MM * TOP_MM_PER_MOTOR_MM);
constexpr uint8_t HOMING_STEPS_PER_SENSOR_READ = 10;

constexpr int LOAD_DETECTED_THRESHOLD = 250;
constexpr int LOAD_RELEASED_THRESHOLD = 350;
constexpr uint8_t LOAD_SENSOR_SAMPLES = 8;
constexpr uint32_t LOAD_STABLE_MS = 250;
constexpr uint32_t LOAD_UPDATE_INTERVAL_MS = 20;
constexpr uint32_t RACK_OFFLINE_MS = 5000;
constexpr uint32_t RACK_REQUEST_MS = 2000;
// Tower command acknowledgement can block the single-threaded web server for up
// to TOWER_ACK_TIMEOUT_MS. Keep enough margin that a healthy browser is not
// mistaken for a disconnected one while an X/Z command is being accepted.
constexpr uint32_t WEB_LEASE_MS = 10000;
constexpr int32_t TRANSFER_Z_STEP_MM = 2;
constexpr int32_t MAX_TRANSFER_Z_MM = 30;
constexpr int32_t X_HOME_MM = 0;
constexpr int32_t Z_HOME_MM = 0;

ASRS_Comm_ESPNow towerCommunication;
ASRS_ESPNow_MasterSession towerSession(towerCommunication);
ASRS_Master towerMaster(towerCommunication);
WebServer server(80);
Preferences preferences;
Adafruit_VL53L0X homeSensor;

struct SavedLocation {
  char name[20];
  int32_t x;
  int32_t y;
  int32_t z;
};

SavedLocation locations[4] = {
  {"Slot 1", 2000, 300, 900}, {"Slot 2", 2000, 300, 1100},
  {"Slot 3", 2200, 300, 900}, {"Slot 4", 2200, 300, 1100}
};
SavedLocation activeTarget = {"Manual", X_HOME_MM, 0, Z_HOME_MM};
bool activeTargetUsesRack = false;

enum OperationType : uint8_t { OP_NONE, OP_PICK, OP_PLACE, OP_POSITION, OP_HOME };
enum SystemState : uint8_t {
  BOOT, HOME_Y_SEARCH, HOME_Y_RETRACT, HOME_TOWER_START, HOME_TOWER_WAIT,
  READY, VALIDATE, PREPARE_RETRACT_Y, MOVE_TOWER_START, MOVE_TOWER_WAIT, EXTEND_Y,
  TRANSFER_CHECK, TRANSFER_MOVE_START, TRANSFER_MOVE_WAIT, RETRACT_Y,
  VERIFY_RACK, RECOVER_RETRACT_Y, RECOVER_TOWER_START, RECOVER_TOWER_WAIT,
  SUCCESS, STOPPING, ERROR_STATE
};

SystemState state = BOOT;
SystemState stoppedFromState = BOOT;
OperationType operation = OP_NONE;
String statusMessage = "Starting";
String errorMessage;
String stopReason;
uint8_t selectedSlot = 0;
bool forkHomed = false;
bool towerHomed = false;
bool towerStationary = false;
bool towerRecoveryRequired = false;
bool loadDetected = false;
int loadSensorValue = 0;
uint32_t loadStateChangedMs = 0;
uint32_t lastLoadUpdateMs = 0;
float currentY = 0;
float targetY = 0;
bool yMoving = false;
bool yDirection = false;
long yStepsRemaining = 0;
uint32_t nextStepUs = 0;
bool stepLevel = false;
long homingSteps = 0;
uint8_t homingReadCounter = 0;
uint32_t lastBrowserHeartbeat = 0;
uint32_t stateStartedMs = 0;
int32_t transferStartZ = 0;
int32_t transferCurrentZ = 0;
ASRS_Coordinates towerCoordinates = {0, 0};
ASRS_Coordinates towerSensorHome = {0, 0};
bool towerCoordinatesSynchronized = false;

volatile bool rackPacketPending = false;
ASRS_RackPacket pendingRackPacket = {};
bool rackOccupied[4] = {false, false, false, false};
bool rackOnline = false;
uint32_t lastRackPacketMs = 0;
uint32_t lastRackRequestMs = 0;
uint32_t lastTowerSessionUpdateMs = 0;
uint32_t lastTowerWaitLogMs = 0;
uint16_t rackSequence = 0;
const uint8_t BROADCAST_MAC[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ASRS Fork Master</title><style>
body{font-family:Arial;background:#eef2f6;margin:0;padding:18px;color:#172033}.wrap{max-width:850px;margin:auto}
.card{background:white;padding:16px;margin:12px 0;border-radius:12px;box-shadow:0 2px 8px #0002}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
.slot{padding:16px;border-radius:8px;color:white;background:#777}.empty{background:#198754}.occupied{background:#c62828}
button,input,select{padding:10px;margin:5px;font-size:15px}button{cursor:pointer}.danger{background:#c62828;color:white}.ok{color:#198754}.bad{color:#c62828}
</style></head><body><div class="wrap"><h1>ASRS Fork Control</h1>
<div class="card"><b id="state">Loading...</b><p id="message"></p><p>X: <span id="x">-</span> mm | Y: <span id="y">-</span> mm | Z: <span id="z">-</span> mm</p>
<p>Load: <span id="load">-</span> | Tower: <span id="tower">-</span> | Rack: <span id="rack">-</span></p>
<button onclick="cmd('/api/home')">Home system</button><button id="recover" onclick="cmd('/api/recover')">Recover tower</button><button class="danger" onclick="cmd('/api/stop')">STOP</button></div>
<div class="card"><h2>Rack</h2><div class="grid" id="slots"></div>
<p><select id="slot"><option value="0">Slot 1</option><option value="1">Slot 2</option><option value="2">Slot 3</option><option value="3">Slot 4</option></select>
<button onclick="operate('pick')">Pick</button><button onclick="operate('place')">Place</button></p></div>
<div class="card"><h2>Independent coordinates</h2><p>Move, pick, or place at a coordinate that is not tied to a rack slot.</p>
<input id="manualX" type="number" min="500" max="2500" value="500" placeholder="X mm"><input id="manualY" type="number" min="-300" max="300" value="0" placeholder="Y mm"><input id="manualZ" type="number" min="400" max="1600" value="400" placeholder="Z mm"><br>
<button onclick="manual('move')">Move to position</button><button onclick="manual('pick')">Pick here</button><button onclick="manual('place')">Place here</button></div>
<div class="card"><h2>Saved coordinates</h2><div id="locations"></div></div></div>
<script>
const state=document.getElementById('state'),message=document.getElementById('message'),x=document.getElementById('x'),y=document.getElementById('y'),z=document.getElementById('z'),load=document.getElementById('load'),tower=document.getElementById('tower'),rack=document.getElementById('rack'),slots=document.getElementById('slots'),locations=document.getElementById('locations'),recover=document.getElementById('recover');
async function cmd(u,o={method:'POST'}){try{let r=await fetch(u,o);let t=await r.text();message.textContent=t;if(!r.ok)state.textContent='REQUEST ERROR'}catch(e){state.textContent='REQUEST ERROR';message.textContent=e.message}}
function operate(t){cmd('/api/'+t+'?slot='+document.getElementById('slot').value)}
function manual(t){let q=new URLSearchParams({type:t,x:document.getElementById('manualX').value,y:document.getElementById('manualY').value,z:document.getElementById('manualZ').value});cmd('/api/manual?'+q)}
async function save(i){let q=new URLSearchParams({slot:i,name:document.getElementById('n'+i).value,x:document.getElementById('x'+i).value,y:document.getElementById('y'+i).value,z:document.getElementById('z'+i).value});cmd('/api/location?'+q)}
async function poll(){try{let d=await(await fetch('/api/status',{cache:'no-store'})).json();
state.textContent=d.state;message.textContent=d.message+(d.error?' - '+d.error:'');x.textContent=d.x;y.textContent=d.y.toFixed(1);z.textContent=d.z;
load.textContent=d.load?'DETECTED':'NOT DETECTED';tower.textContent=d.tower?'CONNECTED':'OFFLINE';rack.textContent=d.rack?'CONNECTED':'OFFLINE';
recover.disabled=!d.recoverable;
slots.innerHTML=d.slots.map((v,i)=>`<div class="slot ${d.rack?(v?'occupied':'empty'):''}">Slot ${i+1}<br>${d.rack?(v?'OCCUPIED':'EMPTY'):'UNKNOWN'}</div>`).join('');
if(!document.getElementById('n0'))locations.innerHTML=d.locations.map((v,i)=>`<p><input id="n${i}" value="${v.name}"><input id="x${i}" type="number" value="${v.x}" placeholder="X"><input id="y${i}" type="number" value="${v.y}" placeholder="Y"><input id="z${i}" type="number" value="${v.z}" placeholder="Z"><button onclick="save(${i})">Save</button></p>`).join('');
await fetch('/api/heartbeat',{method:'POST'});}catch(e){}setTimeout(poll,500)}poll();
</script></body></html>)HTML";

const char *stateName() {
  switch (state) {
    case READY:return "READY"; case SUCCESS:return "SUCCESS"; case ERROR_STATE:return "ERROR";
    case HOME_Y_SEARCH:case HOME_Y_RETRACT:case HOME_TOWER_START:case HOME_TOWER_WAIT:return "HOMING";
    case RECOVER_RETRACT_Y:case RECOVER_TOWER_START:case RECOVER_TOWER_WAIT:return "RECOVERING";
    case STOPPING:return "STOPPING"; default:return "OPERATING";
  }
}

void fail(const String &message) {
  yMoving = false; digitalWrite(STEP_PIN, LOW); stepLevel = false;
  errorMessage = message; statusMessage = "Operation stopped"; state = ERROR_STATE;
}

bool requestStop(const String &reason) {
  if(state==READY || state==SUCCESS || state==ERROR_STATE || state==STOPPING) return false;
  stoppedFromState=state;
  stopReason=reason;
  state=STOPPING;
  return true;
}

void finishStop() {
  const bool yPositionUnknown=yMoving || stoppedFromState==BOOT || stoppedFromState==HOME_Y_SEARCH;
  const bool towerPositionUnknown=towerMaster.operationActive();

  yMoving=false;
  yStepsRemaining=0;
  stepLevel=false;
  digitalWrite(STEP_PIN,LOW);
  digitalWrite(ENABLE_PIN,HIGH);

  if(yPositionUnknown) forkHomed=false;
  if(towerPositionUnknown) {
    towerHomed=false;
    towerStationary=false;
    towerRecoveryRequired=true;
  }

  operation=OP_NONE;
  errorMessage=stopReason.length()?stopReason:"Operation stopped";
  statusMessage=towerPositionUnknown
      ? "Local motion stopped; tower may still be moving"
      : (yPositionUnknown ? "Stopped; Y homing required" : "Operation stopped");
  state=ERROR_STATE;
}

void monitorTowerAfterStop() {
  if(state!=ERROR_STATE || !towerRecoveryRequired || !towerMaster.operationActive()) return;

  ASRS_OperationStatus s;
  if(!towerMaster.readOperationStatus(s)) return;
  towerSession.recordPeerActivity();
  if(s.status==ASRS_STATUS_BUSY) return;

  towerStationary=true;
  towerHomed=false;
  if(s.status==ASRS_STATUS_ERROR)
    errorMessage=String("Stopped operation; tower reported ")+asrsErrorName(s.error);
  statusMessage="Tower is stationary; use Recover tower";
}

bool rawRackReceiver(const uint8_t *, const uint8_t *data, int length) {
  if (!asrsIsRackPacket(data, length)) return false;
  ASRS_RackPacket packet; memcpy(&packet, data, sizeof(packet));
  if (packet.type == ASRS_RACK_STATUS) {
    memcpy(&pendingRackPacket, &packet, sizeof(packet)); rackPacketPending = true;
  }
  return true;
}

void processRackPacket() {
  if (!rackPacketPending) return;
  noInterrupts(); ASRS_RackPacket packet = pendingRackPacket; rackPacketPending = false; interrupts();
  for (uint8_t i=0;i<4;i++) rackOccupied[i] = packet.slots[i] != 0;
  lastRackPacketMs = millis(); rackOnline = true;
}

void requestRackStatus() {
  ASRS_RackPacket p={}; p.magic=ASRS_RACK_MAGIC; p.version=ASRS_RACK_VERSION;
  p.type=ASRS_RACK_STATUS_REQUEST; p.sequence=++rackSequence; p.uptimeMs=millis();
  esp_now_send(BROADCAST_MAC, reinterpret_cast<uint8_t*>(&p), sizeof(p));
}

void updateLoad() {
  uint32_t total=0; for(uint8_t i=0;i<LOAD_SENSOR_SAMPLES;i++) total+=analogRead(LOAD_SENSOR_PIN);
  loadSensorValue=total/LOAD_SENSOR_SAMPLES; bool previous=loadDetected;
  if(!loadDetected && loadSensorValue<=LOAD_DETECTED_THRESHOLD) loadDetected=true;
  else if(loadDetected && loadSensorValue>=LOAD_RELEASED_THRESHOLD) loadDetected=false;
  if(previous!=loadDetected) loadStateChangedMs=millis();
}

bool stableLoad(bool expected) { return loadDetected==expected && millis()-loadStateChangedMs>=LOAD_STABLE_MS; }

bool startYMove(float destination) {
  if(destination<Y_MIN_MM || destination>Y_MAX_MM || yMoving || !forkHomed) return false;
  float delta=destination-currentY; yStepsRemaining=lroundf(fabs(delta)*STEPS_PER_TOP_MM);
  targetY=destination; yDirection=delta>0; digitalWrite(DIR_PIN,yDirection?HIGH:LOW);
  digitalWrite(ENABLE_PIN,LOW); yMoving=yStepsRemaining>0; nextStepUs=micros();
  if(!yMoving) currentY=destination; return true;
}

void updateYMotor() {
  if(!yMoving || (int32_t)(micros()-nextStepUs)<0) return;
  nextStepUs += STEP_HALF_PERIOD_US; stepLevel=!stepLevel; digitalWrite(STEP_PIN,stepLevel);
  if(!stepLevel && --yStepsRemaining<=0) { yMoving=false; currentY=targetY; }
}

void updateYHoming() {
  if(state==HOME_Y_SEARCH) {
    if(!homeSensor.begin()) { fail("VL53L0X homing sensor not detected"); return; }
    digitalWrite(ENABLE_PIN,LOW); digitalWrite(DIR_PIN,HIGH); state=BOOT; homingSteps=0; statusMessage="Searching for Y home sensor";
  }
  if(state!=BOOT || forkHomed) return;
  if(++homingReadCounter>=HOMING_STEPS_PER_SENSOR_READ) {
    homingReadCounter=0; VL53L0X_RangingMeasurementData_t m; homeSensor.rangingTest(&m,false);
    if(m.RangeStatus!=4 && m.RangeMilliMeter<=HOMING_SENSOR_DISTANCE_MM) {
      forkHomed=true; currentY=HOMING_RETRACT_TOP_MM; state=HOME_Y_RETRACT;
      if(!startYMove(0)) fail("Could not retract after Y homing"); return;
    }
  }
  if(homingSteps++>=MAX_HOMING_STEPS) { fail("Y home sensor threshold not reached"); return; }
  digitalWrite(STEP_PIN,HIGH); delayMicroseconds(STEP_HALF_PERIOD_US); digitalWrite(STEP_PIN,LOW); delayMicroseconds(STEP_HALF_PERIOD_US);
}

bool waitTowerTerminal() {
  ASRS_OperationStatus s;
  if(!towerMaster.readOperationStatus(s)) {
    if(millis()-lastTowerWaitLogMs>=TOWER_WAIT_LOG_MS){lastTowerWaitLogMs=millis();Serial.println("Waiting for tower BUSY/DONE/ERROR status");}
    return false;
  }
  towerSession.recordPeerActivity();
  Serial.print("Tower status: ");Serial.println(asrsStatusName(s.status));
  if(s.status==ASRS_STATUS_ERROR) {
    if(s.error==ASRS_ERROR_LIMIT_REACHED) {
      towerStationary=true;
      towerHomed=false;
      towerRecoveryRequired=true;
      operation=OP_NONE;
      errorMessage="Tower limit reached; use Recover tower";
      statusMessage="Tower stopped and requires homing";
      state=ERROR_STATE;
    } else {
      fail(String("Tower error: ")+asrsErrorName(s.error));
    }
    return false;
  }
  if(s.status==ASRS_STATUS_DONE) { towerStationary=true; return true; }
  return false;
}

bool towerOperationTimedOut() {
  if(millis()-stateStartedMs<=TOWER_OPERATION_TIMEOUT_MS) return false;
  fail("Tower operation did not report DONE; reset required");
  return true;
}

bool readTowerCoordinates(bool captureHome) {
  ASRS_Coordinates measured;
  if(!towerMaster.requestCoordinates(measured,TOWER_ACK_TIMEOUT_MS)) {
    fail(String("Could not read tower coordinates: ")+asrsErrorName(towerMaster.lastError()));
    return false;
  }
  towerSession.recordPeerActivity();
  if(captureHome) {
    towerSensorHome=measured;
    towerCoordinates={0,0};
    towerCoordinatesSynchronized=true;
  } else {
    if(!towerCoordinatesSynchronized) {
      fail("Tower coordinate origin has not been captured; home the system");
      return false;
    }
    towerCoordinates={measured.x-towerSensorHome.x,measured.z-towerSensorHome.z};
  }
  Serial.printf("Tower ToF raw X=%ld Z=%ld; logical X=%ld Z=%ld\n",
      (long)measured.x,(long)measured.z,
      (long)towerCoordinates.x,(long)towerCoordinates.z);
  return true;
}

bool sendTowerToAbsolute(int32_t targetX, int32_t targetZ) {
  const int32_t commandX=TOWER_COMMANDS_ARE_RELATIVE
      ? targetX-towerCoordinates.x : targetX+towerSensorHome.x;
  const int32_t commandZ=TOWER_COMMANDS_ARE_RELATIVE
      ? targetZ-towerCoordinates.z : targetZ+towerSensorHome.z;
  Serial.printf("Tower absolute target X=%ld Z=%ld; command X=%ld Z=%ld\n",
      (long)targetX,(long)targetZ,(long)commandX,(long)commandZ);
  return towerMaster.sendTravelCommand(commandX,commandZ,TOWER_ACK_TIMEOUT_MS);
}

bool beginHome() {
  if(state!=READY && state!=ERROR_STATE && state!=SUCCESS && state!=BOOT) return false;
  if(towerMaster.operationActive()) return false;
  operation=OP_HOME; errorMessage=""; forkHomed=false; towerHomed=false;
  towerCoordinatesSynchronized=false; towerRecoveryRequired=false; state=HOME_Y_SEARCH;
  return true;
}

bool beginTowerRecovery() {
  if(state!=ERROR_STATE || !towerRecoveryRequired ||
     towerMaster.operationActive() || !towerSession.connected() ||
     !forkHomed || yMoving) return false;

  operation=OP_HOME;
  errorMessage="";
  statusMessage="Recovering from tower limit stop";

  if(fabs(currentY)>Y_RETRACTED_TOLERANCE_MM) {
    if(!startYMove(0)) return false;
    state=RECOVER_RETRACT_Y;
  } else {
    state=RECOVER_TOWER_START;
  }
  return true;
}

void updateOperation() {
  if(state==HOME_Y_SEARCH || (state==BOOT && !forkHomed)) { updateYHoming(); return; }
  if(state==HOME_Y_RETRACT && !yMoving) { state=HOME_TOWER_START; }
  if(state==HOME_TOWER_START) {
    if(!towerSession.connected()) return;
    Serial.printf("Sending tower homing command; ACK timeout=%lu ms\n",(unsigned long)TOWER_ACK_TIMEOUT_MS);
    if(towerMaster.sendHomingCommand(true,true,TOWER_ACK_TIMEOUT_MS)) { towerSession.recordPeerActivity(); towerStationary=false; stateStartedMs=millis(); Serial.println("Tower homing accepted"); state=HOME_TOWER_WAIT; }
    else { Serial.printf("Tower homing ACK failed: %s\n",asrsErrorName(towerMaster.lastError())); fail(String("Tower homing rejected: ")+asrsErrorName(towerMaster.lastError())); }
  }
  if(state==HOME_TOWER_WAIT && waitTowerTerminal()) {
    if(!readTowerCoordinates(true)) return;
    towerHomed=true; operation=OP_NONE; state=READY; statusMessage="System homed and ready";
  }
  if(state==HOME_TOWER_WAIT && towerOperationTimedOut()) return;
  if(state==VALIDATE) {
    if(!forkHomed||!towerHomed) { fail("System must be homed"); return; }
    if(activeTargetUsesRack && !rackOnline) { fail("Rack status is offline"); return; }
    if(activeTargetUsesRack && operation==OP_PICK && !rackOccupied[selectedSlot]) { fail("Cannot pick from an empty slot"); return; }
    if(activeTargetUsesRack && operation==OP_PLACE && rackOccupied[selectedSlot]) { fail("Cannot place into an occupied slot"); return; }
    if(operation==OP_PICK && loadDetected) { fail("Fork already detects a load"); return; }
    if(operation==OP_PLACE && !loadDetected) { fail("No load detected for place operation"); return; }
    if(fabs(currentY)>Y_RETRACTED_TOLERANCE_MM) {
      if(!startYMove(0)) { fail("Could not retract fork before tower travel"); return; }
      state=PREPARE_RETRACT_Y;
    } else {
      state=MOVE_TOWER_START;
    }
  }
  if(state==PREPARE_RETRACT_Y && !yMoving) { currentY=0; state=MOVE_TOWER_START; }
  if(state==MOVE_TOWER_START) {
    if(sendTowerToAbsolute(activeTarget.x,activeTarget.z)) { towerSession.recordPeerActivity(); towerStationary=false; stateStartedMs=millis(); state=MOVE_TOWER_WAIT; }
    else fail(String("Tower move rejected: ")+asrsErrorName(towerMaster.lastError()));
  }
  if(state==MOVE_TOWER_WAIT && waitTowerTerminal()) {
    if(!readTowerCoordinates(false)) return;
    if(!startYMove(activeTarget.y)) fail("Invalid or unavailable Y movement"); else state=EXTEND_Y;
  }
  if(state==MOVE_TOWER_WAIT && towerOperationTimedOut()) return;
  if(state==EXTEND_Y && !yMoving) {
    if(operation==OP_POSITION) {
      operation=OP_NONE;
      statusMessage="Independent position reached";
      state=SUCCESS;
      return;
    }
    transferStartZ=towerCoordinates.z; transferCurrentZ=transferStartZ; state=TRANSFER_CHECK;
  }
  if(state==TRANSFER_CHECK) {
    bool acquired=operation==OP_PICK ? stableLoad(true) : stableLoad(false);
    if(acquired) { if(!startYMove(0)) fail("Could not retract fork"); else state=RETRACT_Y; return; }
    int32_t travelled=abs(transferCurrentZ-transferStartZ);
    if(travelled>=MAX_TRANSFER_Z_MM) { fail(operation==OP_PICK?"Load not detected within pick lift":"Load not released within place descent"); return; }
    state=TRANSFER_MOVE_START;
  }
  if(state==TRANSFER_MOVE_START) {
    transferCurrentZ += operation==OP_PICK ? TRANSFER_Z_STEP_MM : -TRANSFER_Z_STEP_MM;
    if(sendTowerToAbsolute(towerCoordinates.x,transferCurrentZ)) { towerSession.recordPeerActivity(); towerStationary=false; stateStartedMs=millis(); state=TRANSFER_MOVE_WAIT; }
    else fail("Tower transfer step rejected");
  }
  if(state==TRANSFER_MOVE_WAIT && waitTowerTerminal()) {
    if(!readTowerCoordinates(false)) return;
    transferCurrentZ=towerCoordinates.z;
    state=TRANSFER_CHECK;
  }
  if(state==TRANSFER_MOVE_WAIT && towerOperationTimedOut()) return;
  if(state==RETRACT_Y && !yMoving) {
    if(!activeTargetUsesRack) {
      statusMessage=operation==OP_PICK?"Independent pick completed":"Independent place completed";
      operation=OP_NONE;
      state=SUCCESS;
    } else {
      stateStartedMs=millis(); requestRackStatus(); state=VERIFY_RACK;
    }
  }
  if(state==VERIFY_RACK) {
    bool expected=operation==OP_PLACE;
    if(rackOnline && rackOccupied[selectedSlot]==expected) { statusMessage=operation==OP_PICK?"Pick completed":"Place completed"; operation=OP_NONE; state=SUCCESS; }
    else if(millis()-stateStartedMs>5000) fail("Rack did not confirm occupancy change");
  }
  if(state==RECOVER_RETRACT_Y && !yMoving) {
    currentY=0;
    state=RECOVER_TOWER_START;
  }
  if(state==RECOVER_TOWER_START) {
    if(!towerSession.connected()) return;
    Serial.printf("Sending recovery tower homing command; ACK timeout=%lu ms\n",(unsigned long)TOWER_ACK_TIMEOUT_MS);
    if(towerMaster.sendHomingCommand(true,true,TOWER_ACK_TIMEOUT_MS)) {
      towerSession.recordPeerActivity();
      towerStationary=false;
      stateStartedMs=millis();
      state=RECOVER_TOWER_WAIT;
    } else {
      fail(String("Tower recovery homing rejected: ")+asrsErrorName(towerMaster.lastError()));
    }
  }
  if(state==RECOVER_TOWER_WAIT && waitTowerTerminal()) {
    if(!readTowerCoordinates(true)) return;
    towerHomed=true;
    towerStationary=true;
    towerRecoveryRequired=false;
    operation=OP_NONE;
    errorMessage="";
    statusMessage="Tower recovered and system ready";
    state=READY;
  }
  if(state==RECOVER_TOWER_WAIT && towerOperationTimedOut()) return;
  if(state==STOPPING) finishStop();
}

void sendStatus() {
  // A successful status poll is itself proof that the control page is present.
  // This also avoids a false stop if the separate heartbeat request is delayed.
  lastBrowserHeartbeat=millis();
  String j="{\"state\":\""+String(stateName())+"\",\"message\":\""+statusMessage+"\",\"error\":\""+errorMessage+"\"";
  j+=",\"x\":"+String(towerCoordinates.x)+",\"y\":"+String(currentY,1)+",\"z\":"+String(towerCoordinates.z);
  j+=",\"load\":"+String(loadDetected?"true":"false")+",\"tower\":"+String(towerSession.connected()?"true":"false")+",\"rack\":"+String(rackOnline?"true":"false");
  j+=",\"recoverable\":"+String((state==ERROR_STATE&&towerRecoveryRequired&&towerStationary&&!towerMaster.operationActive()&&towerSession.connected()&&forkHomed&&!yMoving)?"true":"false");
  j+=",\"slots\":["; for(uint8_t i=0;i<4;i++){if(i)j+=",";j+=rackOccupied[i]?"true":"false";} j+="]";
  j+=",\"locations\":["; for(uint8_t i=0;i<4;i++){if(i)j+=",";j+="{\"name\":\""+String(locations[i].name)+"\",\"x\":"+locations[i].x+",\"y\":"+locations[i].y+",\"z\":"+locations[i].z+"}";} j+="]}";
  server.send(200,"application/json",j);
}

void loadLocations() {
  preferences.begin("asrs_locations",false);
  for(uint8_t i=0;i<4;i++){String key="loc"+String(i);preferences.getBytes(key.c_str(),&locations[i],sizeof(SavedLocation));}
}

void setupWeb() {
  WiFi.mode(WIFI_AP_STA); WiFi.setSleep(false); WiFi.softAP(AP_SSID,AP_PASSWORD,ESPNOW_CHANNEL); loadLocations();
  server.on("/",[](){server.send_P(200,"text/html",PAGE);}); server.on("/api/status",HTTP_GET,sendStatus);
  server.on("/api/heartbeat",HTTP_POST,[](){lastBrowserHeartbeat=millis();server.send(200,"text/plain","OK");});
  server.on("/api/home",HTTP_POST,[](){if(beginHome())server.send(202,"text/plain","Homing requested");else server.send(409,"text/plain","Homing already active or system busy");});
  server.on("/api/recover",HTTP_POST,[](){if(beginTowerRecovery())server.send(202,"text/plain","Tower recovery started");else server.send(409,"text/plain","Tower recovery is unavailable");});
  server.on("/api/stop",HTTP_POST,[](){
    if(requestStop("Stopped by operator")) server.send(202,"text/plain","Stopping");
    else server.send(409,"text/plain","No active operation to stop");
  });
  auto operationHandler=[](OperationType requested){
    if(state!=READY&&state!=SUCCESS){server.send(409,"text/plain","System is not ready");return;}
    int slot=server.arg("slot").toInt();if(slot<0||slot>3){server.send(422,"text/plain","Invalid slot");return;}
    selectedSlot=slot;activeTarget=locations[slot];activeTargetUsesRack=true;operation=requested;errorMessage="";statusMessage=requested==OP_PICK?"Rack pick started":"Rack place started";lastBrowserHeartbeat=millis();state=VALIDATE;stateStartedMs=millis();server.send(202,"text/plain","Accepted");
  };
  server.on("/api/pick",HTTP_POST,[operationHandler](){operationHandler(OP_PICK);});
  server.on("/api/place",HTTP_POST,[operationHandler](){operationHandler(OP_PLACE);});
  server.on("/api/manual",HTTP_POST,[](){
    if(state!=READY&&state!=SUCCESS){server.send(409,"text/plain","System is not ready");return;}
    if(!server.hasArg("type")||!server.hasArg("x")||!server.hasArg("y")||!server.hasArg("z")){server.send(400,"text/plain","Missing type or coordinate");return;}
    int32_t targetX=server.arg("x").toInt(),targetY=server.arg("y").toInt(),targetZ=server.arg("z").toInt();
    if(targetX<500||targetX>2500||targetY<-300||targetY>300||targetZ<400||targetZ>1600){server.send(422,"text/plain","Coordinate outside configured limits");return;}
    String type=server.arg("type");
    if(type=="pick")operation=OP_PICK;else if(type=="place")operation=OP_PLACE;else if(type=="move")operation=OP_POSITION;else{server.send(422,"text/plain","Unknown manual operation");return;}
    strncpy(activeTarget.name,"Manual",sizeof(activeTarget.name));activeTarget.name[sizeof(activeTarget.name)-1]='\0';activeTarget.x=targetX;activeTarget.y=targetY;activeTarget.z=targetZ;
    activeTargetUsesRack=false;errorMessage="";statusMessage=type+" to independent coordinate started";lastBrowserHeartbeat=millis();state=VALIDATE;stateStartedMs=millis();server.send(202,"text/plain","Independent coordinate operation accepted");
  });
  server.on("/api/location",HTTP_POST,[](){int i=server.arg("slot").toInt();int x=server.arg("x").toInt(),y=server.arg("y").toInt(),z=server.arg("z").toInt();
    if(i<0||i>3||x<500||x>2500||y<-300||y>300||z<400||z>1600){server.send(422,"text/plain","Coordinate outside configured limits");return;}
    server.arg("name").substring(0,19).toCharArray(locations[i].name,sizeof(locations[i].name));locations[i].x=x;locations[i].y=y;locations[i].z=z;
    String key="loc"+String(i);preferences.putBytes(key.c_str(),&locations[i],sizeof(SavedLocation));server.send(200,"text/plain","Saved");});
  server.begin(); Serial.print("Web UI: http://"); Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200); pinMode(STEP_PIN,OUTPUT);pinMode(DIR_PIN,OUTPUT);pinMode(ENABLE_PIN,OUTPUT);pinMode(LOAD_SENSOR_PIN,INPUT);
  digitalWrite(STEP_PIN,LOW);digitalWrite(ENABLE_PIN,LOW);analogReadResolution(12);Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
  ASRS_Comm_ESPNow::setRawReceiveHandler(rawRackReceiver);
  setupWeb();updateLoad();
  if(!towerSession.begin(ESPNOW_CHANNEL,PAIR_TIMEOUT_MS,&Serial)) Serial.println("Tower not paired yet; session will continue recovery.");
  towerSession.setRepairingTimeout(REPAIR_ATTEMPT_TIMEOUT_MS);
  towerSession.setHeartbeatInterval(0);
  towerSession.setLinkTimeout(0);
  WiFi.mode(WIFI_AP_STA);WiFi.setSleep(false);
  if(!WiFi.softAP(AP_SSID,AP_PASSWORD,ESPNOW_CHANNEL)) Serial.println("ERROR: Fork Wi-Fi AP failed after ESP-NOW startup");
  else {Serial.print("Fork Wi-Fi ready: ");Serial.println(WiFi.softAPIP());}
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,BROADCAST_MAC,6);peer.channel=ESPNOW_CHANNEL;peer.encrypt=false;if(!esp_now_is_peer_exist(BROADCAST_MAC))esp_now_add_peer(&peer);
  beginHome();
}

void loop() {
  server.handleClient();
  if(towerSession.connected()||millis()-lastTowerSessionUpdateMs>=TOWER_SESSION_UPDATE_MS){lastTowerSessionUpdateMs=millis();towerSession.update();}
  processRackPacket(); updateYMotor();
  if(millis()-lastLoadUpdateMs>=LOAD_UPDATE_INTERVAL_MS){lastLoadUpdateMs=millis();updateLoad();}
  if(millis()-lastRackRequestMs>=RACK_REQUEST_MS){lastRackRequestMs=millis();requestRackStatus();}
  if(rackOnline&&millis()-lastRackPacketMs>RACK_OFFLINE_MS)rackOnline=false;
  bool active=state!=READY&&state!=SUCCESS&&state!=ERROR_STATE&&operation!=OP_HOME;
  if(active&&lastBrowserHeartbeat!=0&&millis()-lastBrowserHeartbeat>WEB_LEASE_MS)
    requestStop("Stopped because the control webpage disconnected");
  updateOperation();
  monitorTowerAfterStop();
  delay(1);
}
