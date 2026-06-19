#include <WiFi.h>

#define MQTT_MAX_PACKET_SIZE 768
#include <PubSubClient.h>

#include <ACB_SmartCar_V2.h>
#include <QTRSensors.h>
#include <SPI.h>
#include <MFRC522.h>

// =======================
// WiFi
// =======================
const char *ssid     = "moble_classroom";
const char *password = "moble2025";

// =======================
// MQTT
// =======================
const char *mqtt_broker = "192.168.0.15";
const int   mqtt_port   = 1883;
const char *mqtt_client_id = "AGV_1";

const char *TOPIC_STATUS  = "visipick/agv/1/status";
const char *TOPIC_COMMAND = "visipick/agv/1/command";
const char *TOPIC_RFID_UID = "visipick/agv/1/rfid";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// =======================
// RFID RC522
// =======================
#define RFID_SS_PIN    5
#define RFID_RST_PIN   32
#define RFID_SCK_PIN   18
#define RFID_MISO_PIN  19
#define RFID_MOSI_PIN  21

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);

// =======================
// RFID UID
// =======================
const char *UID_NODE_START         = "D6:B9:39:F4";  // 출발점으로 들어오는 AGV를 멈추게 하는 RFID
const char *UID_NODE_START_TURN_STOP = "3B:47:43:5A"; // 출발점 마지막 회전 후 멈추는 RFID
const char *UID_NODE_HOME_JUNCTION = "3B:47:3D:5A";
const char *UID_NODE_JUNCTION      = "3B:47:42:5A";
const char *UID_NODE_SELECT_LEFT   = "E7:C0:3B:27";
const char *UID_NODE_SELECT_RIGHT  = "3B:47:44:5A";
const char *UID_NODE_WAREHOUSE_1   = "3B:47:45:5A";
const char *UID_NODE_WAREHOUSE_2   = "2A:7A:61:E1";

const unsigned long RFID_SAME_UID_IGNORE_MS = 2500;
const unsigned long RFID_READ_GAP_MS = 250;

unsigned long lastRfidReadMs = 0;
unsigned long lastSameUidMs = 0;
String lastRfidUid = "";

// =======================
// 교차로 설정
// =======================
const unsigned long JUNCTION_LEFT_TRACK_MS  = 320;
const unsigned long JUNCTION_RIGHT_TRACK_MS = 320;

const int JUNCTION_LEFT_INNER_SPEED = -150;
const int JUNCTION_LEFT_OUTER_SPEED = 220;

const int JUNCTION_RIGHT_OUTER_SPEED = 220;
const int JUNCTION_RIGHT_RF_SPEED    = -180;
const int JUNCTION_RIGHT_RR_SPEED    = -150;

const int LINE_CORNER_OUTER_SPEED = 220;
const int LINE_CORNER_INNER_SPEED = -150;

// =======================
// 자동 복귀 설정
// =======================
const unsigned long WAREHOUSE_WAIT_MS = 5000;
const unsigned long RETURN_TURN_AROUND_MS = 1600;

// 출발점 도착 후 마지막 한바퀴 회전만 약 15도 더 돌림
// 기존 1600ms 기준 15도 ≒ 67ms라서 70ms 추가
const unsigned long START_FINAL_TURN_EXTRA_MS = 70;
const unsigned long START_FINAL_TURN_MS = RETURN_TURN_AROUND_MS + START_FINAL_TURN_EXTRA_MS;

// 출발점 마지막 회전은 RFID로 정지한다.
// 너무 빨리 같은 위치 근처 태그를 읽는 것을 막기 위한 최소 무시 시간과,
// RFID 미인식 시 계속 도는 것을 막기 위한 안전 타임아웃
const unsigned long START_FINAL_RFID_IGNORE_MS = 500;
const unsigned long START_FINAL_RFID_TIMEOUT_MS = 8000;

// 홈에 있던 AGV가 시작점으로 빠질 때 사용
const unsigned long START_FINAL_WAIT_MS = 1500;

// 출발점 마지막 회전 속도만 천천히
const int START_FINAL_TURN_SPEED = 150;

// 홈1/홈3에서 시작점 복귀 시, 한바퀴 회전 후 조금 전진하고 커브
const unsigned long START_HOME1_BEFORE_RIGHT_MS = 4000;
const unsigned long START_HOME3_BEFORE_LEFT_MS  = 3900;
const unsigned long START_HOME1_EXIT_RIGHT_MS   = 650;
const unsigned long START_HOME3_EXIT_LEFT_MS    = 650;

// 복귀 분기 감지값
// 너무 낮으면 일찍 꺾고, 너무 높으면 인식 못 함
const int RETURN_BRANCH_DETECT_THRESHOLD = 60;
const int RETURN_ERROR_DETECT = 600;

const unsigned long RETURN_BRANCH_LEFT_TRACK_MS  = 650;
const unsigned long RETURN_BRANCH_RIGHT_TRACK_MS = 650;

unsigned long returnTrackingStartMs = 0;
const unsigned long RETURN_BRANCH_IGNORE_MS = 1000;

// 창고2 복귀 오른쪽 감지만 아주 조금 늦게 감지
const unsigned long RETURN_RIGHT_BRANCH_EXTRA_IGNORE_MS = 150;

// 창고 일자 테이프 정지용
bool warehouseStopArmed = false;
int warehouseStopTarget = 0;
unsigned long warehouseStopArmedAt = 0;
const unsigned long WAREHOUSE_STOP_IGNORE_MS = 800;

int returnFromWarehouse = 0;
int selectedHome = 0;
int startReturnFromHome = 0;

unsigned long returnWaitStartMs = 0;
unsigned long returnTurnStartMs = 0;

unsigned long startFinalWaitStartMs = 0;
unsigned long startFinalTurnStartMs = 0;
unsigned long startHomeBeforeCurveStartMs = 0;
unsigned long startHomeExitTurnStartMs = 0;

bool homeFree1 = true;
bool homeFree2 = true;
bool homeFree3 = true;

bool homeStopArmed = false;
unsigned long homeStopArmedAt = 0;
const unsigned long HOME_STOP_IGNORE_MS = 1200;

bool returnBranchReady = false;
bool returnBranchTurnActive = false;

// =======================
// Objects
// =======================
ACB_SmartCar_V2 ACB_SmartCar;
QTRSensors qtr;

// =======================
// QTR
// =======================
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const uint8_t sensorPins[SensorCount] = {
  4, 12, 23, 25, 26, 27, 14, 33
};

int floorBase[SensorCount];
int lineValue[SensorCount];

int HIT_THRESHOLD = 80;
int FLOOR_MARGIN = 20;

const int MIN_LINE_SUM = 80;

// =======================
// Mode / Status
// =======================
enum FUNCTION_MODE {
  STANDBY,
  TRACKING
};

FUNCTION_MODE function_mode = STANDBY;

enum AGV_STATUS {
  AGV_STANDBY,
  AGV_TRACKING,
  AGV_OBSTACLE_STOP,
  AGV_LINE_LOST,
  AGV_EMERGENCY
};

AGV_STATUS agv_status = AGV_STANDBY;
bool emergencyStop = false;
bool trayLoaded = false;

enum MISSION_TARGET {
  MISSION_NONE,
  MISSION_WAREHOUSE_1,
  MISSION_WAREHOUSE_2,
  MISSION_HOME_1,
  MISSION_HOME_2,
  MISSION_HOME_3,
  MISSION_START
};

MISSION_TARGET missionTarget = MISSION_NONE;

enum NEXT_ACTION {
  ACTION_NONE,
  ACTION_WAIT_RFID,
  ACTION_TURN_LEFT,
  ACTION_TURN_RIGHT,
  ACTION_FOLLOW_LINE,
  ACTION_ARRIVED_WAREHOUSE_1,
  ACTION_ARRIVED_WAREHOUSE_2,
  ACTION_ARRIVED_HOME,
  ACTION_RETURN_HOME,
  ACTION_RETURN_START,
  ACTION_ARRIVED_START
};

NEXT_ACTION nextAction = ACTION_NONE;

enum NODE_STATE {
  NODE_NONE,
  NODE_START,
  NODE_START_TURN_STOP,
  NODE_HOME_JUNCTION,
  NODE_JUNCTION,
  NODE_SELECT_LEFT,
  NODE_SELECT_RIGHT,
  NODE_WAREHOUSE_1,
  NODE_WAREHOUSE_2,
  NODE_UNKNOWN
};

NODE_STATE currentNode = NODE_NONE;
int currentNodeId = 0;

enum JUNCTION_TURN_MODE {
  JTURN_NONE,
  JTURN_LEFT_TRACK,
  JTURN_RIGHT_TRACK
};

JUNCTION_TURN_MODE junctionTurnMode = JTURN_NONE;
unsigned long junctionTurnStartMs = 0;

enum RETURN_MODE {
  RETURN_NONE,
  RETURN_WAIT_5SEC,
  RETURN_TURN_AROUND,
  RETURN_TRACKING_FROM_W1,
  RETURN_TRACKING_FROM_W2,
  RETURN_HOME_SELECT,
  RETURN_NO_FREE_HOME_WAIT,
  RETURN_HOME_STOP,
  RETURN_TO_START_TURN_AROUND,
  RETURN_TO_START_BEFORE_EXIT_CURVE,
  RETURN_TO_START_EXIT_RIGHT,
  RETURN_TO_START_EXIT_LEFT,
  RETURN_TO_START_TRACKING,
  RETURN_START_FINAL_WAIT,
  RETURN_START_FINAL_TURN,
  RETURN_START_TO_WAREHOUSE_TURN_AROUND
};

RETURN_MODE returnMode = RETURN_NONE;

// =======================
// PID / Motor
// =======================
int baseSpeed = 200;

float KP = 0.055;
float KD = 0.13;

const int MIN_MOTOR_SPEED = 0;
const int MAX_MOTOR_SPEED = 225;

const int MAX_CORRECTION = 130;
const int MAX_SPEED_DIFF = 255;

const int FILTER_OLD = 50;
const int FILTER_NEW = 50;

const int SLEW_STEP = 30;

const int CENTER_POS = 3500;
const int DEAD_BAND = 50;

const bool PID_REVERSE = false;

const int STRAIGHT_CENTER_SUM = 180;
const int STRAIGHT_EDGE_MAX = 180;
const int STRAIGHT_ERROR_LIMIT = 900;

const int STRAIGHT_DEAD_BAND = 280;
const float STRAIGHT_KP = 0.006;
const int STRAIGHT_MAX_CORRECTION = 8;

const int LEFT_TRIM = 0;
const int RIGHT_TRIM = 0;
const int RF_TRIM = 0;

const unsigned long CURVE_RECOVERY_MS = 180;
unsigned long lastForceTurnMs = 0;

const int CORNER_EDGE_SUM = 100;
const int CORNER_CENTER_WEAK = 180;
const int CORNER_MARGIN = 35;

const int FORCE_TURN_ERROR = 2200;

// =======================
// Ultrasonic
// =======================
const int ULTRASONIC_TRIG_PIN = 13;
const int ULTRASONIC_ECHO_PIN = 34;

const int OBSTACLE_STOP_CM = 15;
const int OBSTACLE_CLEAR_CM = 20;

const unsigned long ULTRASONIC_INTERVAL_MS = 80;
const unsigned long ULTRASONIC_TIMEOUT_US = 5000;

const int OBSTACLE_CONFIRM_COUNT = 2;
const int OBSTACLE_CLEAR_COUNT = 2;

const bool USE_ULTRASONIC_STOP = true;

// =======================
// Debug
// =======================
bool debugQTR = false;
bool debugUS = true;
bool debugRFID = true;

int currentLF = 0;
int currentLR = 0;
int currentRF = 0;
int currentRR = 0;

void stopAllMotors();
void publishStatus(long distCm = -1);
void publishRfidUid(const String &uid);
void startJunctionTurn(JUNCTION_TURN_MODE turnMode);
bool handleRfidNode();

// =======================
// 문자열 함수
// =======================
const char* missionStr() {
  switch (missionTarget) {
    case MISSION_WAREHOUSE_1: return "GO_WAREHOUSE_1";
    case MISSION_WAREHOUSE_2: return "GO_WAREHOUSE_2";
    case MISSION_HOME_1: return "GO_HOME_1";
    case MISSION_HOME_2: return "GO_HOME_2";
    case MISSION_HOME_3: return "GO_HOME_3";
    case MISSION_START: return "GO_START";
    default: return "NONE";
  }
}

const char* nextActionStr() {
  switch (nextAction) {
    case ACTION_WAIT_RFID: return "WAIT_RFID";
    case ACTION_TURN_LEFT: return "TURN_LEFT";
    case ACTION_TURN_RIGHT: return "TURN_RIGHT";
    case ACTION_FOLLOW_LINE: return "FOLLOW_LINE";
    case ACTION_ARRIVED_WAREHOUSE_1: return "ARRIVED_WAREHOUSE_1";
    case ACTION_ARRIVED_WAREHOUSE_2: return "ARRIVED_WAREHOUSE_2";
    case ACTION_ARRIVED_HOME: return "ARRIVED_HOME";
    case ACTION_RETURN_HOME: return "RETURN_HOME";
    case ACTION_RETURN_START: return "RETURN_START";
    case ACTION_ARRIVED_START: return "ARRIVED_START";
    default: return "NONE";
  }
}

const char* nodeStr() {
  switch (currentNode) {
    case NODE_START: return "START";
    case NODE_START_TURN_STOP: return "START_TURN_STOP";
    case NODE_HOME_JUNCTION: return "HOME_JUNCTION";
    case NODE_JUNCTION: return "JUNCTION";
    case NODE_SELECT_LEFT: return "SELECT_DONE_LEFT";
    case NODE_SELECT_RIGHT: return "SELECT_DONE_RIGHT";
    case NODE_WAREHOUSE_1: return "WAREHOUSE_1";
    case NODE_WAREHOUSE_2: return "WAREHOUSE_2";
    case NODE_UNKNOWN: return "UNKNOWN";
    default: return "NONE";
  }
}

const char* agvStatusStr() {
  if (emergencyStop) return "EMERGENCY";

  switch (agv_status) {
    case AGV_TRACKING: return "TRACKING";
    case AGV_OBSTACLE_STOP: return "OBSTACLE_STOP";
    case AGV_LINE_LOST: return "LINE_LOST";
    default: return "STANDBY";
  }
}

const char* returnModeStr() {
  switch (returnMode) {
    case RETURN_WAIT_5SEC: return "WAIT_5SEC";
    case RETURN_TURN_AROUND: return "TURN_AROUND";
    case RETURN_TRACKING_FROM_W1: return "RETURN_FROM_W1";
    case RETURN_TRACKING_FROM_W2: return "RETURN_FROM_W2";
    case RETURN_HOME_SELECT: return "HOME_SELECT";
    case RETURN_NO_FREE_HOME_WAIT: return "NO_FREE_HOME_WAIT";
    case RETURN_HOME_STOP: return "HOME_STOP";
    case RETURN_TO_START_TURN_AROUND: return "TO_START_TURN_AROUND";
    case RETURN_TO_START_BEFORE_EXIT_CURVE: return "TO_START_BEFORE_EXIT_CURVE";
    case RETURN_TO_START_EXIT_RIGHT: return "TO_START_EXIT_RIGHT";
    case RETURN_TO_START_EXIT_LEFT: return "TO_START_EXIT_LEFT";
    case RETURN_TO_START_TRACKING: return "TO_START_TRACKING";
    case RETURN_START_FINAL_WAIT: return "START_FINAL_WAIT";
    case RETURN_START_FINAL_TURN: return "START_FINAL_TURN";
    case RETURN_START_TO_WAREHOUSE_TURN_AROUND: return "START_TO_WAREHOUSE_TURN_AROUND";
    default: return "NONE";
  }
}

// =======================
// 홈 선택
// =======================
int selectFreeHome() {
  if (homeFree1) return 1;
  if (homeFree2) return 2;
  if (homeFree3) return 3;
  return 0;
}

void markSelectedHomeBusy() {
  if (selectedHome == 1) homeFree1 = false;
  if (selectedHome == 2) homeFree2 = false;
  if (selectedHome == 3) homeFree3 = false;
}

bool tryStartHomeAfterFree() {
  if (returnMode != RETURN_NO_FREE_HOME_WAIT) {
    return false;
  }

  selectedHome = selectFreeHome();

  if (selectedHome == 0) {
    Serial.println("[HOME] STILL NO FREE HOME → WAIT");
    publishStatus();
    return true;
  }

  function_mode = TRACKING;
  agv_status = AGV_TRACKING;
  returnMode = RETURN_HOME_STOP;
  nextAction = ACTION_FOLLOW_LINE;

  if (selectedHome == 1) {
    homeStopArmed = false;
    startJunctionTurn(JTURN_LEFT_TRACK);

    Serial.println("[HOME] FREE DETECTED → HOME1 → LEFT");
    return true;
  }

  if (selectedHome == 2) {
    junctionTurnMode = JTURN_NONE;
    homeStopArmed = true;
    homeStopArmedAt = millis();

    Serial.println("[HOME] FREE DETECTED → HOME2 → STRAIGHT → STOP LINE ARMED AFTER 1.2SEC");
    publishStatus();
    return true;
  }

  if (selectedHome == 3) {
    homeStopArmed = false;
    startJunctionTurn(JTURN_RIGHT_TRACK);

    Serial.println("[HOME] FREE DETECTED → HOME3 → RIGHT");
    return true;
  }

  return false;
}

// =======================
// RFID
// =======================
String uidToString(MFRC522::Uid *uid) {
  String uidStr = "";

  for (byte i = 0; i < uid->size; i++) {
    if (uid->uidByte[i] < 0x10) {
      uidStr += "0";
    }

    uidStr += String(uid->uidByte[i], HEX);

    if (i < uid->size - 1) {
      uidStr += ":";
    }
  }

  uidStr.toUpperCase();
  return uidStr;
}

NODE_STATE getNodeFromUid(String uid) {
  uid.toUpperCase();

  if (uid == UID_NODE_START) {
    currentNodeId = 0;
    return NODE_START;
  }

  if (uid == UID_NODE_START_TURN_STOP) {
    currentNodeId = 7;
    return NODE_START_TURN_STOP;
  }

  if (uid == UID_NODE_HOME_JUNCTION) {
    currentNodeId = 1;
    return NODE_HOME_JUNCTION;
  }

  if (uid == UID_NODE_JUNCTION) {
    currentNodeId = 2;
    return NODE_JUNCTION;
  }

  if (uid == UID_NODE_SELECT_LEFT) {
    currentNodeId = 3;
    return NODE_SELECT_LEFT;
  }

  if (uid == UID_NODE_SELECT_RIGHT) {
    currentNodeId = 4;
    return NODE_SELECT_RIGHT;
  }

  if (uid == UID_NODE_WAREHOUSE_1) {
    currentNodeId = 5;
    return NODE_WAREHOUSE_1;
  }

  if (uid == UID_NODE_WAREHOUSE_2) {
    currentNodeId = 6;
    return NODE_WAREHOUSE_2;
  }

  currentNodeId = 0;
  return NODE_UNKNOWN;
}

// =======================
// MQTT RFID UID 전송
// =======================
void publishRfidUid(const String &uid) {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT RFID TX] FAIL - not connected");
    return;
  }

  char buf[96];

  snprintf(
    buf,
    sizeof(buf),
    "{\"agv_id\":\"AGV_1\",\"rfid_uid\":\"%s\"}",
    uid.c_str()
  );

  bool ok = mqttClient.publish(TOPIC_RFID_UID, buf);

  Serial.print("[MQTT RFID TX] ");
  Serial.print(ok ? "OK " : "FAIL ");
  Serial.println(buf);
}

// =======================
// MQTT 상태 전송
// =======================
void publishStatus(long distCm) {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT TX] FAIL - not connected");
    return;
  }

  char buf[760];

  snprintf(
    buf,
    sizeof(buf),
    "{\"agv_id\":\"AGV_1\",\"status\":\"%s\",\"tray_loaded\":%s,"
    "\"mission\":\"%s\",\"node_id\":%d,\"node\":\"%s\","
    "\"next_action\":\"%s\",\"rfid_uid\":\"%s\",\"dist_cm\":%ld,"
    "\"return_mode\":\"%s\",\"return_from\":%d,\"branch_ready\":%s,"
    "\"branch_turn_active\":%s,\"selected_home\":%d,"
    "\"warehouse_stop_armed\":%s,\"warehouse_stop_target\":%d,"
    "\"home1_free\":%s,\"home2_free\":%s,\"home3_free\":%s,"
    "\"home_stop_armed\":%s}",
    agvStatusStr(),
    trayLoaded ? "true" : "false",
    missionStr(),
    currentNodeId,
    nodeStr(),
    nextActionStr(),
    lastRfidUid.c_str(),
    distCm,
    returnModeStr(),
    returnFromWarehouse,
    returnBranchReady ? "true" : "false",
    returnBranchTurnActive ? "true" : "false",
    selectedHome,
    warehouseStopArmed ? "true" : "false",
    warehouseStopTarget,
    homeFree1 ? "true" : "false",
    homeFree2 ? "true" : "false",
    homeFree3 ? "true" : "false",
    homeStopArmed ? "true" : "false"
  );

  bool ok = mqttClient.publish(TOPIC_STATUS, buf);

  Serial.print("[MQTT TX] ");
  Serial.print(ok ? "OK " : "FAIL ");
  Serial.println(buf);
}

// =======================
// Motor
// =======================
int avoidDeadSpeed(int speed) {
  if (speed == 0) return 0;

  int s = abs(speed);

  if (s >= 1 && s <= 125) {
    s = 130;
  }

  if (speed < 0) return -s;
  return s;
}

int smoothSpeed(int current, int target) {
  if (target > current + SLEW_STEP) return current + SLEW_STEP;
  if (target < current - SLEW_STEP) return current - SLEW_STEP;
  return target;
}

void setFourMotor(int lf, int lr, int rf, int rr, bool smooth = true) {
  rf += RF_TRIM;

  lf = constrain(lf, -255, 255);
  lr = constrain(lr, -255, 255);
  rf = constrain(rf, -255, 255);
  rr = constrain(rr, -255, 255);

  lf = avoidDeadSpeed(lf);
  lr = avoidDeadSpeed(lr);
  rf = avoidDeadSpeed(rf);
  rr = avoidDeadSpeed(rr);

  if (smooth) {
    currentLF = smoothSpeed(currentLF, lf);
    currentLR = smoothSpeed(currentLR, lr);
    currentRF = smoothSpeed(currentRF, rf);
    currentRR = smoothSpeed(currentRR, rr);
  } else {
    currentLF = lf;
    currentLR = lr;
    currentRF = rf;
    currentRR = rr;
  }

  currentLF = avoidDeadSpeed(currentLF);
  currentLR = avoidDeadSpeed(currentLR);
  currentRF = avoidDeadSpeed(currentRF);
  currentRR = avoidDeadSpeed(currentRR);

  ACB_SmartCar.motorControl(1, currentLF);
  ACB_SmartCar.motorControl(2, currentLR);
  ACB_SmartCar.motorControl(3, currentRF);
  ACB_SmartCar.motorControl(4, currentRR);
}

void stopAllMotors() {
  currentLF = 0;
  currentLR = 0;
  currentRF = 0;
  currentRR = 0;

  ACB_SmartCar.motorControl(1, 0);
  ACB_SmartCar.motorControl(2, 0);
  ACB_SmartCar.motorControl(3, 0);
  ACB_SmartCar.motorControl(4, 0);
  ACB_SmartCar.Move(Stop, 0);
}

void forceTurnLeft() {
  lastForceTurnMs = millis();

  setFourMotor(
    LINE_CORNER_INNER_SPEED,
    LINE_CORNER_INNER_SPEED,
    LINE_CORNER_OUTER_SPEED,
    LINE_CORNER_OUTER_SPEED,
    false
  );
}

void forceTurnRight() {
  lastForceTurnMs = millis();

  setFourMotor(
    LINE_CORNER_OUTER_SPEED,
    LINE_CORNER_OUTER_SPEED,
    -180,
    LINE_CORNER_INNER_SPEED,
    false
  );
}

void setCurveMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  const int MIN_RUN = 130;

  int slower = min(leftSpeed, rightSpeed);

  if (slower > 0 && slower < MIN_RUN) {
    int boost = MIN_RUN - slower;
    leftSpeed = min(leftSpeed + boost, MAX_MOTOR_SPEED);
    rightSpeed = min(rightSpeed + boost, MAX_MOTOR_SPEED);
  }

  int diff = leftSpeed - rightSpeed;

  if (diff > MAX_SPEED_DIFF) {
    leftSpeed = rightSpeed + MAX_SPEED_DIFF;
  }

  if (diff < -MAX_SPEED_DIFF) {
    rightSpeed = leftSpeed + MAX_SPEED_DIFF;
  }

  leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  setFourMotor(leftSpeed, leftSpeed, rightSpeed, rightSpeed, true);
}

// =======================
// QTR
// =======================
void calibrateWhiteFloor() {
  stopAllMotors();

  Serial.println();
  Serial.println("================================");
  Serial.println("White floor calibration");
  Serial.println("Keep sensor on white floor");
  Serial.println("Start after 2 seconds");
  Serial.println("================================");

  delay(2000);

  long sum[SensorCount];

  for (int i = 0; i < SensorCount; i++) {
    sum[i] = 0;
  }

  for (int n = 0; n < 100; n++) {
    qtr.read(sensorValues);

    for (int i = 0; i < SensorCount; i++) {
      sum[i] += sensorValues[i];
    }

    delay(10);
  }

  Serial.print("BASE: ");

  for (int i = 0; i < SensorCount; i++) {
    floorBase[i] = sum[i] / 100;
    Serial.print(floorBase[i]);
    Serial.print("\t");
  }

  Serial.println();
  Serial.println("BASE OK");
}

bool readLineSensor(long &weightedSum, long &totalValue, int &maxValue, int &hitCount) {
  qtr.read(sensorValues);

  weightedSum = 0;
  totalValue = 0;
  maxValue = 0;
  hitCount = 0;

  for (int i = 0; i < SensorCount; i++) {
    int v = (int)sensorValues[i] - floorBase[i] - FLOOR_MARGIN;

    if (v < 0) v = 0;

    lineValue[i] = v;

    weightedSum += (long)v * i * 1000;
    totalValue += v;

    if (v > maxValue) maxValue = v;
    if (v >= HIT_THRESHOLD) hitCount++;
  }

  if (maxValue >= HIT_THRESHOLD && totalValue >= MIN_LINE_SUM) {
    return true;
  }

  return false;
}

// 홈 도착 테이프용: 민감하게
bool homeStopLineDetected(int hitCount, long totalValue) {
  if (hitCount >= 4 && totalValue >= 700) {
    return true;
  }

  return false;
}

// 창고 도착 테이프용: 옆길 오인 방지 때문에 빡세게
bool warehouseStopLineDetected(int hitCount, long totalValue) {
  if (hitCount >= 6 && totalValue >= 1200) {
    return true;
  }

  return false;
}

// =======================
// Ultrasonic
// =======================
long readUltrasonicDistanceCm() {
  unsigned long waitStart = micros();

  while (digitalRead(ULTRASONIC_ECHO_PIN) == HIGH) {
    if (micros() - waitStart > 2000) break;
  }

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );

  if (duration == 0) return 999;

  long distanceCm = duration / 58;

  if (distanceCm < 2 || distanceCm > 200) {
    return 999;
  }

  return distanceCm;
}

bool obstacleDetected() {
  static unsigned long lastMeasureTime = 0;
  static long distanceCm = 999;

  static int obstacleCount = 0;
  static int clearCount = 0;

  static bool obstacleHold = false;
  static unsigned long lastPrintTime = 0;

  if (millis() - lastMeasureTime >= ULTRASONIC_INTERVAL_MS) {
    lastMeasureTime = millis();

    distanceCm = readUltrasonicDistanceCm();

    if (distanceCm != 999) {
      if (distanceCm > 0 && distanceCm <= OBSTACLE_STOP_CM) {
        obstacleCount++;
        clearCount = 0;
      }
      else if (distanceCm >= OBSTACLE_CLEAR_CM) {
        clearCount++;
        obstacleCount = 0;
      }
      else {
        obstacleCount = 0;
        clearCount = 0;
      }
    }
    else {
      clearCount++;
      obstacleCount = 0;
    }

    if (obstacleCount >= OBSTACLE_CONFIRM_COUNT) {
      obstacleHold = true;
    }

    if (clearCount >= OBSTACLE_CLEAR_COUNT) {
      obstacleHold = false;
    }

    if (debugUS && millis() - lastPrintTime >= 300) {
      lastPrintTime = millis();

      Serial.print("[US] DIST=");
      Serial.print(distanceCm);
      Serial.print("cm HOLD=");
      Serial.println(obstacleHold ? "ON" : "OFF");
    }
  }

  return obstacleHold;
}

// =======================
// 교차로 특수제어
// =======================
void startJunctionTurn(JUNCTION_TURN_MODE turnMode) {
  junctionTurnMode = turnMode;
  junctionTurnStartMs = millis();

  if (turnMode == JTURN_LEFT_TRACK) {
    nextAction = ACTION_TURN_LEFT;
    Serial.println("[JUNCTION] LEFT TRACK START");
  }
  else if (turnMode == JTURN_RIGHT_TRACK) {
    nextAction = ACTION_TURN_RIGHT;
    Serial.println("[JUNCTION] RIGHT TRACK START");
  }

  publishStatus();
}

bool processJunctionTurn() {
  if (junctionTurnMode == JTURN_NONE) {
    return false;
  }

  unsigned long elapsed = millis() - junctionTurnStartMs;

  if (junctionTurnMode == JTURN_LEFT_TRACK) {
    setFourMotor(
      JUNCTION_LEFT_INNER_SPEED,
      JUNCTION_LEFT_INNER_SPEED,
      JUNCTION_LEFT_OUTER_SPEED,
      JUNCTION_LEFT_OUTER_SPEED,
      false
    );

    unsigned long targetMs =
      returnBranchTurnActive ? RETURN_BRANCH_LEFT_TRACK_MS : JUNCTION_LEFT_TRACK_MS;

    if (elapsed >= targetMs) {
      junctionTurnMode = JTURN_NONE;
      nextAction = ACTION_FOLLOW_LINE;
      lastForceTurnMs = millis();

      if (returnBranchTurnActive) {
        returnBranchTurnActive = false;
        returnMode = RETURN_HOME_SELECT;
        homeStopArmed = false;

        Serial.println("[RETURN] BRANCH LEFT TURN DONE → FIND RFID1 HOME_JUNCTION");
      }
      else {
        if (returnMode == RETURN_HOME_STOP ||
            missionTarget == MISSION_HOME_1) {
          homeStopArmed = true;
          homeStopArmedAt = millis();
        }

        if (missionTarget == MISSION_WAREHOUSE_1 &&
            returnMode == RETURN_NONE) {
          warehouseStopArmed = true;
          warehouseStopTarget = 1;
          warehouseStopArmedAt = millis();

          Serial.println("[WAREHOUSE] W1 STOP TAPE ARMED");
        }

        Serial.println("[JUNCTION] LEFT TRACK DONE → NORMAL TRACKING");
      }

      publishStatus();

      delay(4);
      return false;
    }

    delay(6);
    return true;
  }

  if (junctionTurnMode == JTURN_RIGHT_TRACK) {
    setFourMotor(
      JUNCTION_RIGHT_OUTER_SPEED,
      JUNCTION_RIGHT_OUTER_SPEED,
      JUNCTION_RIGHT_RF_SPEED,
      JUNCTION_RIGHT_RR_SPEED,
      false
    );

    unsigned long targetMs =
      returnBranchTurnActive ? RETURN_BRANCH_RIGHT_TRACK_MS : JUNCTION_RIGHT_TRACK_MS;

    if (elapsed >= targetMs) {
      junctionTurnMode = JTURN_NONE;
      nextAction = ACTION_FOLLOW_LINE;
      lastForceTurnMs = millis();

      if (returnBranchTurnActive) {
        returnBranchTurnActive = false;
        returnMode = RETURN_HOME_SELECT;
        homeStopArmed = false;

        Serial.println("[RETURN] BRANCH RIGHT TURN DONE → FIND RFID1 HOME_JUNCTION");
      }
      else {
        if (returnMode == RETURN_HOME_STOP ||
            missionTarget == MISSION_HOME_3) {
          homeStopArmed = true;
          homeStopArmedAt = millis();
        }

        if (missionTarget == MISSION_WAREHOUSE_2 &&
            returnMode == RETURN_NONE) {
          warehouseStopArmed = true;
          warehouseStopTarget = 2;
          warehouseStopArmedAt = millis();

          Serial.println("[WAREHOUSE] W2 STOP TAPE ARMED");
        }

        Serial.println("[JUNCTION] RIGHT TRACK DONE → NORMAL TRACKING");
      }

      publishStatus();

      delay(4);
      return false;
    }

    delay(6);
    return true;
  }

  return false;
}

// =======================
// 자동 복귀 처리
// =======================
bool processReturnMode() {
  if (returnMode == RETURN_NONE) {
    return false;
  }

  if (returnMode == RETURN_WAIT_5SEC) {
    stopAllMotors();

    if (millis() - returnWaitStartMs >= WAREHOUSE_WAIT_MS) {
      returnMode = RETURN_TURN_AROUND;
      returnTurnStartMs = millis();
      nextAction = ACTION_RETURN_HOME;

      Serial.println("[RETURN] 5SEC DONE → ONE FULL TURN START");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_TURN_AROUND) {
    setFourMotor(
      220,
      220,
      -220,
      -220,
      false
    );

    if (millis() - returnTurnStartMs >= RETURN_TURN_AROUND_MS) {
      if (returnFromWarehouse == 1) {
        returnMode = RETURN_TRACKING_FROM_W1;
      }
      else if (returnFromWarehouse == 2) {
        returnMode = RETURN_TRACKING_FROM_W2;
      }
      else {
        returnMode = RETURN_NONE;
      }

      returnBranchReady = false;
      returnBranchTurnActive = false;
      returnTrackingStartMs = millis();
      warehouseStopArmed = false;
      warehouseStopTarget = 0;

      function_mode = TRACKING;
      agv_status = AGV_TRACKING;
      nextAction = ACTION_RETURN_HOME;
      homeStopArmed = false;

      Serial.println("[RETURN] ONE FULL TURN DONE → RETURN TRACKING");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_START_TO_WAREHOUSE_TURN_AROUND) {
    setFourMotor(
      220,
      220,
      -220,
      -220,
      false
    );

    if (millis() - returnTurnStartMs >= RETURN_TURN_AROUND_MS) {
      returnMode = RETURN_NONE;
      returnFromWarehouse = 0;
      startReturnFromHome = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;

      function_mode = TRACKING;
      agv_status = AGV_TRACKING;
      nextAction = ACTION_WAIT_RFID;
      lastForceTurnMs = millis();

      Serial.println("[START] TRAYS READY → ONE FULL TURN DONE → GO WAREHOUSE TRACKING");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_TO_START_TURN_AROUND) {
    setFourMotor(
      220,
      220,
      -220,
      -220,
      false
    );

    if (millis() - returnTurnStartMs >= RETURN_TURN_AROUND_MS) {
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;

      function_mode = TRACKING;
      agv_status = AGV_TRACKING;

      if (startReturnFromHome == 1 || startReturnFromHome == 3) {
        returnMode = RETURN_TO_START_BEFORE_EXIT_CURVE;
        startHomeBeforeCurveStartMs = millis();
        nextAction = ACTION_FOLLOW_LINE;

        Serial.println("[START_RETURN] ONE FULL TURN DONE → TRACKING UNTIL STOP TAPE");
      }
      else {
        returnMode = RETURN_TO_START_TRACKING;
        returnTrackingStartMs = millis();
        nextAction = ACTION_RETURN_START;

        Serial.println("[START_RETURN] ONE FULL TURN DONE → TRACKING TO START");
      }

      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_TO_START_BEFORE_EXIT_CURVE) {
    return false;
  }

  if (returnMode == RETURN_TO_START_EXIT_RIGHT) {
    setFourMotor(
      JUNCTION_RIGHT_OUTER_SPEED,
      JUNCTION_RIGHT_OUTER_SPEED,
      JUNCTION_RIGHT_RF_SPEED,
      JUNCTION_RIGHT_RR_SPEED,
      false
    );

    if (millis() - startHomeExitTurnStartMs >= START_HOME1_EXIT_RIGHT_MS) {
      returnMode = RETURN_TO_START_TRACKING;
      returnTrackingStartMs = millis();
      nextAction = ACTION_RETURN_START;
      lastForceTurnMs = millis();

      Serial.println("[START_RETURN] HOME1 RIGHT CURVE DONE → TRACKING TO START");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_TO_START_EXIT_LEFT) {
    setFourMotor(
      JUNCTION_LEFT_INNER_SPEED,
      JUNCTION_LEFT_INNER_SPEED,
      JUNCTION_LEFT_OUTER_SPEED,
      JUNCTION_LEFT_OUTER_SPEED,
      false
    );

    if (millis() - startHomeExitTurnStartMs >= START_HOME3_EXIT_LEFT_MS) {
      returnMode = RETURN_TO_START_TRACKING;
      returnTrackingStartMs = millis();
      nextAction = ACTION_RETURN_START;
      lastForceTurnMs = millis();

      Serial.println("[START_RETURN] HOME3 LEFT CURVE DONE → TRACKING TO START");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_START_FINAL_WAIT) {
    stopAllMotors();

    if (millis() - startFinalWaitStartMs >= START_FINAL_WAIT_MS) {
      returnMode = RETURN_START_FINAL_TURN;
      startFinalTurnStartMs = millis();
      nextAction = ACTION_ARRIVED_START;

      Serial.println("[START_RETURN] START POINT STOP 1SEC DONE → FINAL TURN UNTIL RFID");
      publishStatus();
    }

    return true;
  }

  if (returnMode == RETURN_START_FINAL_TURN) {
    setFourMotor(
      START_FINAL_TURN_SPEED,
      START_FINAL_TURN_SPEED,
      -START_FINAL_TURN_SPEED,
      -START_FINAL_TURN_SPEED,
      false
    );

    handleRfidNode();

    if (returnMode != RETURN_START_FINAL_TURN) {
      return true;
    }

    if (millis() - startFinalTurnStartMs >= START_FINAL_RFID_TIMEOUT_MS) {
      stopAllMotors();

      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      nextAction = ACTION_ARRIVED_START;
      trayLoaded = false;

      returnMode = RETURN_NONE;
      returnFromWarehouse = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      selectedHome = 0;
      startReturnFromHome = 0;

      Serial.println("[START_RETURN] FINAL STOP RFID TIMEOUT → STOP AT START");
      publishStatus();
    }

    return true;
  }

  return false;
}

// =======================
// RFID Node 처리
// =======================
bool handleRfidNode() {
  if (millis() - lastRfidReadMs < RFID_READ_GAP_MS) {
    return false;
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    return false;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return false;
  }

  String uid = uidToString(&rfid.uid);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  publishRfidUid(uid);

  unsigned long now = millis();

  if (uid == lastRfidUid && now - lastSameUidMs < RFID_SAME_UID_IGNORE_MS) {
    return false;
  }

  lastRfidUid = uid;
  lastSameUidMs = now;
  lastRfidReadMs = now;

  currentNode = getNodeFromUid(uid);

  if (debugRFID) {
    Serial.print("[RFID] UID=");
    Serial.print(uid);
    Serial.print(" NODE_ID=");
    Serial.print(currentNodeId);
    Serial.print(" NODE=");
    Serial.println(nodeStr());
  }

  if (currentNode == NODE_START_TURN_STOP) {
    if (returnMode == RETURN_START_FINAL_TURN) {
      if (millis() - startFinalTurnStartMs < START_FINAL_RFID_IGNORE_MS) {
        Serial.println("[START_RETURN] FINAL STOP RFID ignored too early");
        publishStatus();
        return false;
      }

      stopAllMotors();

      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      nextAction = ACTION_ARRIVED_START;
      trayLoaded = false;

      returnMode = RETURN_NONE;
      returnFromWarehouse = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      selectedHome = 0;
      startReturnFromHome = 0;

      Serial.println("[START_RETURN] FINAL STOP RFID detected → STOP AT START");
      publishStatus();
      return false;
    }

    Serial.println("[RFID] START FINAL STOP RFID detected → IGNORE EXCEPT FINAL TURN");
    publishStatus();
    return false;
  }

  if (currentNode == NODE_START) {
    if (returnMode == RETURN_TO_START_TRACKING) {
      stopAllMotors();

      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      nextAction = ACTION_ARRIVED_START;
      trayLoaded = false;

      returnMode = RETURN_NONE;
      returnFromWarehouse = 0;
      startReturnFromHome = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      selectedHome = 0;

      Serial.println("[START_RETURN] START POINT detected → STOP AND WAIT TRAYS");
      publishStatus();
      return false;
    }

    Serial.println("[RFID] START POINT detected → publish start position");
    publishStatus();
    return false;
  }

  if (currentNode == NODE_HOME_JUNCTION) {
    if (returnMode == RETURN_TO_START_TRACKING ||
        returnMode == RETURN_TO_START_BEFORE_EXIT_CURVE) {
      nextAction = ACTION_FOLLOW_LINE;
      Serial.println("[START_RETURN] HOME_JUNCTION during START RETURN → STRAIGHT TRACKING");
      publishStatus();
      return false;
    }

    int targetHome = 0;

    if (returnMode == RETURN_HOME_SELECT) {
      selectedHome = selectFreeHome();
      targetHome = selectedHome;
    }
    else if (missionTarget == MISSION_HOME_1) {
      selectedHome = 1;
      targetHome = 1;
    }
    else if (missionTarget == MISSION_HOME_2) {
      selectedHome = 2;
      targetHome = 2;
    }
    else if (missionTarget == MISSION_HOME_3) {
      selectedHome = 3;
      targetHome = 3;
    }

    if (targetHome == 1) {
      homeStopArmed = false;
      returnMode = RETURN_HOME_STOP;
      startJunctionTurn(JTURN_LEFT_TRACK);

      Serial.println("[HOME] HOME_JUNCTION → HOME1 → LEFT");
      return true;
    }

    if (targetHome == 2) {
      junctionTurnMode = JTURN_NONE;
      nextAction = ACTION_FOLLOW_LINE;
      homeStopArmed = true;
      homeStopArmedAt = millis();
      returnMode = RETURN_HOME_STOP;

      Serial.println("[HOME] HOME_JUNCTION → HOME2 → STRAIGHT → STOP LINE ARMED AFTER 1.2SEC");
      publishStatus();
      return false;
    }

    if (targetHome == 3) {
      homeStopArmed = false;
      returnMode = RETURN_HOME_STOP;
      startJunctionTurn(JTURN_RIGHT_TRACK);

      Serial.println("[HOME] HOME_JUNCTION → HOME3 → RIGHT");
      return true;
    }

    if (returnMode == RETURN_HOME_SELECT) {
      Serial.println("[HOME] NO FREE HOME → WAIT AT RFID1");
      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      returnMode = RETURN_NO_FREE_HOME_WAIT;
      nextAction = ACTION_WAIT_RFID;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      stopAllMotors();
      publishStatus();
      return false;
    }

    Serial.println("[RFID] HOME_JUNCTION detected, but no home mission");
    publishStatus();
    return false;
  }

  if (currentNode == NODE_JUNCTION) {
    if (returnMode != RETURN_NONE) {
      nextAction = ACTION_FOLLOW_LINE;
      Serial.println("[RFID] JUNCTION during RETURN → STRAIGHT TRACKING");
      publishStatus();
      return false;
    }

    if (missionTarget == MISSION_WAREHOUSE_1) {
      warehouseStopArmed = false;
      warehouseStopTarget = 0;

      startJunctionTurn(JTURN_LEFT_TRACK);
      Serial.println("[RFID] JUNCTION → WAREHOUSE_1 → LEFT");
      return true;
    }

    if (missionTarget == MISSION_WAREHOUSE_2) {
      warehouseStopArmed = false;
      warehouseStopTarget = 0;

      startJunctionTurn(JTURN_RIGHT_TRACK);
      Serial.println("[RFID] JUNCTION → WAREHOUSE_2 → RIGHT");
      return true;
    }

    if (missionTarget == MISSION_HOME_1 ||
        missionTarget == MISSION_HOME_2 ||
        missionTarget == MISSION_HOME_3) {
      junctionTurnMode = JTURN_NONE;
      nextAction = ACTION_FOLLOW_LINE;

      Serial.println("[RFID] JUNCTION → HOME MISSION → STRAIGHT TRACKING");
      publishStatus();
      return false;
    }

    Serial.println("[RFID] JUNCTION detected, but mission is NONE");
    nextAction = ACTION_NONE;
    publishStatus();
    return false;
  }

  if (currentNode == NODE_SELECT_LEFT) {
    nextAction = ACTION_FOLLOW_LINE;
    Serial.println("[RFID] SELECT_LEFT → IGNORE");
    publishStatus();
    return false;
  }

  if (currentNode == NODE_SELECT_RIGHT) {
    nextAction = ACTION_FOLLOW_LINE;
    Serial.println("[RFID] SELECT_RIGHT → IGNORE");
    publishStatus();
    return false;
  }

  if (currentNode == NODE_WAREHOUSE_1) {
    if (returnMode != RETURN_NONE) {
      Serial.println("[RFID] WAREHOUSE_1 ignored during RETURN");
      publishStatus();
      return false;
    }

    if (missionTarget == MISSION_WAREHOUSE_1) {
      nextAction = ACTION_ARRIVED_WAREHOUSE_1;
      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      trayLoaded = false;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      returnFromWarehouse = 1;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      selectedHome = 0;

      stopAllMotors();

      returnMode = RETURN_WAIT_5SEC;
      returnWaitStartMs = millis();

      Serial.println("[RFID] ARRIVED WAREHOUSE 1 → WAIT 5SEC → ONE FULL TURN → RETURN HOME");
      publishStatus();
      return false;
    }
  }

  if (currentNode == NODE_WAREHOUSE_2) {
    if (returnMode != RETURN_NONE) {
      Serial.println("[RFID] WAREHOUSE_2 ignored during RETURN");
      publishStatus();
      return false;
    }

    if (missionTarget == MISSION_WAREHOUSE_2) {
      nextAction = ACTION_ARRIVED_WAREHOUSE_2;
      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      trayLoaded = false;
      homeStopArmed = false;
      junctionTurnMode = JTURN_NONE;
      returnFromWarehouse = 2;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      selectedHome = 0;

      stopAllMotors();

      returnMode = RETURN_WAIT_5SEC;
      returnWaitStartMs = millis();

      Serial.println("[RFID] ARRIVED WAREHOUSE 2 → WAIT 5SEC → ONE FULL TURN → RETURN HOME");
      publishStatus();
      return false;
    }
  }

  nextAction = ACTION_FOLLOW_LINE;
  publishStatus();
  return false;
}

// =======================
// Tracking
// =======================
void trackingMode() {
  if (emergencyStop) {
    stopAllMotors();
    return;
  }

  if (USE_ULTRASONIC_STOP && obstacleDetected()) {
    agv_status = AGV_OBSTACLE_STOP;
    stopAllMotors();
    delay(20);
    return;
  }

  if (agv_status == AGV_OBSTACLE_STOP) {
    agv_status = AGV_TRACKING;
  }

  if (processJunctionTurn()) {
    return;
  }

  if (handleRfidNode()) {
    delay(4);
    return;
  }

  long weightedSum = 0;
  long totalValue = 0;
  int maxValue = 0;
  int hitCount = 0;

  bool lineDetected = readLineSensor(
    weightedSum,
    totalValue,
    maxValue,
    hitCount
  );

  // =======================
  // 창고 일자 테이프 정지
  // =======================
  if (warehouseStopArmed &&
      lineDetected &&
      millis() - warehouseStopArmedAt >= WAREHOUSE_STOP_IGNORE_MS &&
      warehouseStopLineDetected(hitCount, totalValue)) {
    function_mode = STANDBY;
    agv_status = AGV_STANDBY;
    trayLoaded = false;
    homeStopArmed = false;
    junctionTurnMode = JTURN_NONE;
    returnBranchReady = false;
    returnBranchTurnActive = false;

    if (warehouseStopTarget == 1) {
      nextAction = ACTION_ARRIVED_WAREHOUSE_1;
      returnFromWarehouse = 1;
      Serial.println("[WAREHOUSE] W1 STOP TAPE DETECTED → WAIT 5SEC → RETURN");
    }
    else if (warehouseStopTarget == 2) {
      nextAction = ACTION_ARRIVED_WAREHOUSE_2;
      returnFromWarehouse = 2;
      Serial.println("[WAREHOUSE] W2 STOP TAPE DETECTED → WAIT 5SEC → RETURN");
    }
    else {
      nextAction = ACTION_NONE;
      returnFromWarehouse = 0;
      Serial.println("[WAREHOUSE] STOP TAPE DETECTED BUT TARGET UNKNOWN");
    }

    warehouseStopArmed = false;
    warehouseStopTarget = 0;

    stopAllMotors();

    if (returnFromWarehouse == 1 || returnFromWarehouse == 2) {
      returnMode = RETURN_WAIT_5SEC;
      returnWaitStartMs = millis();
    }

    publishStatus();
    delay(100);
    return;
  }

  // =======================
  // 홈 일자 테이프 정지
  // =======================
  if (homeStopArmed &&
      lineDetected &&
      millis() - homeStopArmedAt >= HOME_STOP_IGNORE_MS &&
      homeStopLineDetected(hitCount, totalValue)) {
    function_mode = STANDBY;
    agv_status = AGV_STANDBY;
    nextAction = ACTION_ARRIVED_HOME;
    trayLoaded = false;
    homeStopArmed = false;
    junctionTurnMode = JTURN_NONE;
    returnMode = RETURN_NONE;
    returnFromWarehouse = 0;
    returnBranchReady = false;
    returnBranchTurnActive = false;
    warehouseStopArmed = false;
    warehouseStopTarget = 0;

    markSelectedHomeBusy();

    stopAllMotors();
    Serial.println("[HOME] FINAL STOP TAPE DETECTED → ARRIVED HOME");
    publishStatus();

    delay(100);
    return;
  }

  // =======================
  // 홈1/홈3 → 출발점 복귀 중 일자 테이프 커브
  // =======================
  if (returnMode == RETURN_TO_START_BEFORE_EXIT_CURVE &&
      lineDetected &&
      (startReturnFromHome == 1 || startReturnFromHome == 3) &&
      millis() - startHomeBeforeCurveStartMs >= HOME_STOP_IGNORE_MS &&
      homeStopLineDetected(hitCount, totalValue)) {
    if (startReturnFromHome == 1) {
      returnMode = RETURN_TO_START_EXIT_RIGHT;
      startHomeExitTurnStartMs = millis();
      nextAction = ACTION_TURN_RIGHT;

      Serial.println("[START_RETURN] HOME1 STOP TAPE DETECTED → EXIT RIGHT CURVE");
      publishStatus();

      delay(8);
      return;
    }

    if (startReturnFromHome == 3) {
      returnMode = RETURN_TO_START_EXIT_LEFT;
      startHomeExitTurnStartMs = millis();
      nextAction = ACTION_TURN_LEFT;

      Serial.println("[START_RETURN] HOME3 STOP TAPE DETECTED → EXIT LEFT CURVE");
      publishStatus();

      delay(8);
      return;
    }
  }

  static int lastError = 0;
  static int filteredError = 0;
  static int lostCount = 0;
  static bool hasSeenLine = false;

  int position = CENTER_POS;
  int error = 0;

  if (lineDetected) {
    hasSeenLine = true;
    lostCount = 0;

    position = weightedSum / totalValue;
    error = position - CENTER_POS;

    int leftEdge = lineValue[0] + lineValue[1];
    int center = lineValue[3] + lineValue[4];
    int rightEdge = lineValue[6] + lineValue[7];

    int returnLeftDetect  = lineValue[0] + lineValue[1] + lineValue[2];
    int returnRightDetect = lineValue[5] + lineValue[6] + lineValue[7];

    bool leftCorner =
      leftEdge >= CORNER_EDGE_SUM &&
      center <= CORNER_CENTER_WEAK &&
      leftEdge > rightEdge + CORNER_MARGIN;

    bool rightCorner =
      rightEdge >= CORNER_EDGE_SUM &&
      center <= CORNER_CENTER_WEAK &&
      rightEdge > leftEdge + CORNER_MARGIN;

    if (returnMode == RETURN_TRACKING_FROM_W1) {
      bool returnBranchDetectEnabled =
        millis() - returnTrackingStartMs >= RETURN_BRANCH_IGNORE_MS;

      if (returnBranchDetectEnabled &&
          (returnLeftDetect >= RETURN_BRANCH_DETECT_THRESHOLD ||
           error <= -RETURN_ERROR_DETECT)) {
        returnBranchTurnActive = true;

        Serial.print("[RETURN] W1 RETURN → LEFT DETECTED = ");
        Serial.print(returnLeftDetect);
        Serial.print(" ERROR=");
        Serial.println(error);

        startJunctionTurn(JTURN_LEFT_TRACK);

        delay(8);
        return;
      }
    }
    else if (returnMode == RETURN_TRACKING_FROM_W2) {
      bool returnBranchDetectEnabled =
        millis() - returnTrackingStartMs >=
        (RETURN_BRANCH_IGNORE_MS + RETURN_RIGHT_BRANCH_EXTRA_IGNORE_MS);

      if (returnBranchDetectEnabled &&
          (returnRightDetect >= RETURN_BRANCH_DETECT_THRESHOLD ||
           error >= RETURN_ERROR_DETECT)) {
        returnBranchTurnActive = true;

        Serial.print("[RETURN] W2 RETURN → RIGHT DETECTED = ");
        Serial.print(returnRightDetect);
        Serial.print(" ERROR=");
        Serial.println(error);

        startJunctionTurn(JTURN_RIGHT_TRACK);

        delay(8);
        return;
      }
    }
    else if (returnMode == RETURN_TO_START_TRACKING ||
             returnMode == RETURN_TO_START_BEFORE_EXIT_CURVE) {
      // 시작점 복귀 중에는 옆 가지 라인 때문에 강제 좌/우 커브를 하지 않고
      // 기존 PID 라인트레이싱으로만 진행한다.
    }
    else {
      if (leftCorner || error <= -FORCE_TURN_ERROR) {
        forceTurnLeft();

        lastError = -2000;
        filteredError = -2000;

        delay(8);
        return;
      }

      if (rightCorner || error >= FORCE_TURN_ERROR) {
        forceTurnRight();

        lastError = 2000;
        filteredError = 2000;

        delay(8);
        return;
      }
    }

    bool straightZone =
      center >= STRAIGHT_CENTER_SUM &&
      leftEdge <= STRAIGHT_EDGE_MAX &&
      rightEdge <= STRAIGHT_EDGE_MAX &&
      abs(error) <= STRAIGHT_ERROR_LIMIT;

    if (straightZone) {
      filteredError = 0;
      lastError = 0;

      bool justAfterCurve =
        lastForceTurnMs > 0 &&
        millis() - lastForceTurnMs <= CURVE_RECOVERY_MS;

      if (justAfterCurve) {
        setFourMotor(
          baseSpeed,
          baseSpeed,
          baseSpeed,
          baseSpeed,
          false
        );

        delay(4);
        return;
      }

      int straightCorrection = 0;

      if (abs(error) > STRAIGHT_DEAD_BAND) {
        straightCorrection = (int)(STRAIGHT_KP * error);

        straightCorrection = constrain(
          straightCorrection,
          -STRAIGHT_MAX_CORRECTION,
          STRAIGHT_MAX_CORRECTION
        );
      }

      int leftSpeed = baseSpeed + straightCorrection + LEFT_TRIM;
      int rightSpeed = baseSpeed - straightCorrection + RIGHT_TRIM;

      setFourMotor(
        leftSpeed,
        leftSpeed,
        rightSpeed,
        rightSpeed,
        false
      );

      delay(4);
      return;
    }

    if (abs(error) < DEAD_BAND) {
      error = 0;
    }

    filteredError = (filteredError * FILTER_OLD + error * FILTER_NEW) / 100;

    int derivative = filteredError - lastError;

    lastError = filteredError;

    int correction = (int)(KP * filteredError + KD * derivative);

    correction = constrain(
      correction,
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

    if (PID_REVERSE) {
      correction = -correction;
    }

    int leftSpeed = baseSpeed + correction + LEFT_TRIM;
    int rightSpeed = baseSpeed - correction + RIGHT_TRIM;

    setCurveMotor(leftSpeed, rightSpeed);
  }
  else {
    lostCount++;

    if (!hasSeenLine) {
      stopAllMotors();
      delay(5);
      return;
    }

    if (lostCount > 300) {
      agv_status = AGV_LINE_LOST;
      function_mode = STANDBY;
      hasSeenLine = false;
      lostCount = 0;

      stopAllMotors();
      return;
    }

    if (lastError > 0) {
      forceTurnRight();
    } else {
      forceTurnLeft();
    }
  }

  delay(4);
}

// =======================
// MQTT Callback
// =======================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String msg;

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  msg.trim();

  Serial.print("[MQTT RX] ");
  Serial.println(msg);

  if (msg == "EMERGENCY_STOP") {
    emergencyStop = true;
    function_mode = STANDBY;
    agv_status = AGV_EMERGENCY;
    junctionTurnMode = JTURN_NONE;
    returnMode = RETURN_NONE;
    returnFromWarehouse = 0;
    startReturnFromHome = 0;
    returnBranchReady = false;
    returnBranchTurnActive = false;
    homeStopArmed = false;
    warehouseStopArmed = false;
    warehouseStopTarget = 0;
    stopAllMotors();
    publishStatus();
    return;
  }

  if (msg == "EMERGENCY_CLEAR") {
    emergencyStop = false;
    agv_status = AGV_STANDBY;
    publishStatus();
    return;
  }

  if (msg == "GO_WAREHOUSE_1") {
    missionTarget = MISSION_WAREHOUSE_1;
    nextAction = ACTION_WAIT_RFID;
    returnMode = RETURN_NONE;
    returnFromWarehouse = 0;
    returnBranchReady = false;
    returnBranchTurnActive = false;
    warehouseStopArmed = false;
    warehouseStopTarget = 0;
    selectedHome = 0;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "GO_WAREHOUSE_2") {
    missionTarget = MISSION_WAREHOUSE_2;
    nextAction = ACTION_WAIT_RFID;
    returnMode = RETURN_NONE;
    returnFromWarehouse = 0;
    returnBranchReady = false;
    returnBranchTurnActive = false;
    warehouseStopArmed = false;
    warehouseStopTarget = 0;
    selectedHome = 0;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "GO_HOME_1") {
    missionTarget = MISSION_HOME_1;
    nextAction = ACTION_WAIT_RFID;
    returnMode = RETURN_NONE;
    selectedHome = 1;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "GO_HOME_2") {
    missionTarget = MISSION_HOME_2;
    nextAction = ACTION_WAIT_RFID;
    returnMode = RETURN_NONE;
    selectedHome = 2;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "GO_HOME_3") {
    missionTarget = MISSION_HOME_3;
    nextAction = ACTION_WAIT_RFID;
    returnMode = RETURN_NONE;
    selectedHome = 3;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "LEAVE_HOME1_TO_START" ||
      msg == "LEAVE_HOME2_TO_START" ||
      msg == "LEAVE_HOME3_TO_START" ||
      msg == "LEAVE_HOME_TO_START" ||
      msg == "GO_START") {
    if (!emergencyStop) {
      if (msg == "LEAVE_HOME1_TO_START") {
        startReturnFromHome = 1;
      }
      else if (msg == "LEAVE_HOME3_TO_START") {
        startReturnFromHome = 3;
      }
      else {
        startReturnFromHome = 2;
      }

      missionTarget = MISSION_START;
      trayLoaded = false;

      function_mode = STANDBY;
      agv_status = AGV_STANDBY;

      junctionTurnMode = JTURN_NONE;
      returnMode = RETURN_TO_START_TURN_AROUND;
      returnTurnStartMs = millis();

      returnFromWarehouse = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;

      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      selectedHome = 0;

      nextAction = ACTION_RETURN_START;

      stopAllMotors();

      Serial.print("[START_RETURN] COMMAND RECEIVED FROM HOME");
      Serial.print(startReturnFromHome);
      Serial.println(" → ONE FULL TURN → TRACKING TO START");
      publishStatus();
    }

    return;
  }

  if (msg == "HOME1_FREE") {
    homeFree1 = true;

    if (tryStartHomeAfterFree()) {
      return;
    }

    publishStatus();
    return;
  }

  if (msg == "HOME1_BUSY") {
    homeFree1 = false;
    publishStatus();
    return;
  }

  if (msg == "HOME2_FREE") {
    homeFree2 = true;

    if (tryStartHomeAfterFree()) {
      return;
    }

    publishStatus();
    return;
  }

  if (msg == "HOME2_BUSY") {
    homeFree2 = false;
    publishStatus();
    return;
  }

  if (msg == "HOME3_FREE") {
    homeFree3 = true;

    if (tryStartHomeAfterFree()) {
      return;
    }

    publishStatus();
    return;
  }

  if (msg == "HOME3_BUSY") {
    homeFree3 = false;
    publishStatus();
    return;
  }

  if (msg == "CLEAR_MISSION") {
    missionTarget = MISSION_NONE;
    nextAction = ACTION_NONE;
    currentNode = NODE_NONE;
    currentNodeId = 0;
    lastRfidUid = "";
    junctionTurnMode = JTURN_NONE;
    returnMode = RETURN_NONE;
    returnFromWarehouse = 0;
    startReturnFromHome = 0;
    returnBranchReady = false;
    returnBranchTurnActive = false;
    warehouseStopArmed = false;
    warehouseStopTarget = 0;
    selectedHome = 0;
    homeStopArmed = false;
    publishStatus();
    return;
  }

  if (msg == "TRAYS_READY_3") {
    if (!emergencyStop) {
      trayLoaded = true;

      if (missionTarget == MISSION_WAREHOUSE_1 ||
          missionTarget == MISSION_WAREHOUSE_2) {
        function_mode = STANDBY;
        agv_status = AGV_STANDBY;

        junctionTurnMode = JTURN_NONE;
        returnMode = RETURN_START_TO_WAREHOUSE_TURN_AROUND;
        returnTurnStartMs = millis();

        returnFromWarehouse = 0;
        startReturnFromHome = 0;
        returnBranchReady = false;
        returnBranchTurnActive = false;

        warehouseStopArmed = false;
        warehouseStopTarget = 0;
        homeStopArmed = false;
        selectedHome = 0;

        nextAction = ACTION_WAIT_RFID;

        stopAllMotors();

        Serial.println("[START] TRAYS_READY_3 → ONE FULL TURN → GO WAREHOUSE");
      }
      else {
        Serial.println("[START] TRAYS_READY_3 received, but warehouse mission is not selected");
      }

      publishStatus();
    }

    return;
  }

  if (msg == "TRAY_LOADED") {
    if (!emergencyStop) {
      trayLoaded = true;
      function_mode = TRACKING;
      agv_status = AGV_TRACKING;

      if (missionTarget == MISSION_WAREHOUSE_1 ||
          missionTarget == MISSION_WAREHOUSE_2 ||
          missionTarget == MISSION_HOME_1 ||
          missionTarget == MISSION_HOME_2 ||
          missionTarget == MISSION_HOME_3) {
        nextAction = ACTION_WAIT_RFID;
      }

      stopAllMotors();
      publishStatus();
    }
    return;
  }

  if (msg == "STATUS") {
    publishStatus();
    return;
  }

  if (msg == "STOP") {
    if (!emergencyStop) {
      function_mode = STANDBY;
      agv_status = AGV_STANDBY;
      junctionTurnMode = JTURN_NONE;
      returnMode = RETURN_NONE;
      returnFromWarehouse = 0;
      startReturnFromHome = 0;
      returnBranchReady = false;
      returnBranchTurnActive = false;
      warehouseStopArmed = false;
      warehouseStopTarget = 0;
      homeStopArmed = false;
      nextAction = ACTION_NONE;
      stopAllMotors();
      publishStatus();
    }
    return;
  }
}

// =======================
// MQTT Reconnect
// =======================
void mqttReconnect() {
  if (mqttClient.connected()) return;

  static unsigned long lastAttempt = 0;

  if (millis() - lastAttempt < 5000) return;

  lastAttempt = millis();

  Serial.print("[MQTT] Connecting...");

  if (mqttClient.connect(mqtt_client_id)) {
    Serial.println(" connected");
    mqttClient.subscribe(TOPIC_COMMAND);
  } else {
    Serial.print(" failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// =======================
// Serial Command
// =======================
void checkSerialCommand() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == 'C' || c == 'c') {
    function_mode = STANDBY;
    calibrateWhiteFloor();
  }
  else if (c == 'D' || c == 'd') {
    debugQTR = !debugQTR;
    Serial.print("[DEBUG_QTR] ");
    Serial.println(debugQTR ? "ON" : "OFF");
  }
  else if (c == 'U' || c == 'u') {
    debugUS = !debugUS;
    Serial.print("[DEBUG_US] ");
    Serial.println(debugUS ? "ON" : "OFF");
  }
  else if (c == 'R' || c == 'r') {
    debugRFID = !debugRFID;
    Serial.print("[DEBUG_RFID] ");
    Serial.println(debugRFID ? "ON" : "OFF");
  }
  else if (c == '1') {
    missionTarget = MISSION_WAREHOUSE_1;
    nextAction = ACTION_WAIT_RFID;
    publishStatus();
  }
  else if (c == '2') {
    missionTarget = MISSION_WAREHOUSE_2;
    nextAction = ACTION_WAIT_RFID;
    publishStatus();
  }
  else if (c == '7') {
    missionTarget = MISSION_HOME_1;
    selectedHome = 1;
    nextAction = ACTION_WAIT_RFID;
    publishStatus();
  }
  else if (c == '8') {
    missionTarget = MISSION_HOME_2;
    selectedHome = 2;
    nextAction = ACTION_WAIT_RFID;
    publishStatus();
  }
  else if (c == '9') {
    missionTarget = MISSION_HOME_3;
    selectedHome = 3;
    nextAction = ACTION_WAIT_RFID;
    publishStatus();
  }
  else if (c == '+') {
    HIT_THRESHOLD += 10;
    Serial.print("HIT_THRESHOLD = ");
    Serial.println(HIT_THRESHOLD);
  }
  else if (c == '-') {
    HIT_THRESHOLD -= 10;

    if (HIT_THRESHOLD < 10) {
      HIT_THRESHOLD = 10;
    }

    Serial.print("HIT_THRESHOLD = ");
    Serial.println(HIT_THRESHOLD);
  }
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");

  unsigned long wifiStart = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection FAILED");
  }

  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setBufferSize(768);
  mqttClient.setCallback(mqttCallback);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setTimeout(2500);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  SPI.begin(
    RFID_SCK_PIN,
    RFID_MISO_PIN,
    RFID_MOSI_PIN,
    RFID_SS_PIN
  );

  rfid.PCD_Init();
  delay(100);

  Serial.print("[RFID] Reader Version: 0x");
  byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    Serial.println("[RFID] RC522 not detected. Check wiring.");
  } else {
    Serial.println("[RFID] RC522 Init OK");
  }

  ACB_SmartCar.Init();

  function_mode = STANDBY;
  stopAllMotors();

  Serial.println("[READY] ESP32 AGV READY");

  calibrateWhiteFloor();

  Serial.println("[MODE] STANDBY");
}

void loop() {
  checkSerialCommand();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(100);
    return;
  }

  if (!mqttClient.connected()) {
    mqttReconnect();
  }

  mqttClient.loop();

  if (processReturnMode()) {
    return;
  }

  if (function_mode == TRACKING) {
    trackingMode();
  } else {
    handleRfidNode();
    stopAllMotors();
  }
}