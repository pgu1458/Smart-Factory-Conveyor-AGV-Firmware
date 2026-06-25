[README.md](https://github.com/user-attachments/files/29124225/README.md)
#  실시간 영상을 분석한 제품 자동 정렬 및 분류 시스템 — 컨베이어·AGV 펌웨어

> 비전 분류 라인의 **하드웨어/펌웨어 전 영역**을 담당한 ESP32 펌웨어 모음
> 컨베이어·게이트·AGV 라인트레이싱·RFID 노드·자동 충전 복귀 + NC/NO 이중 비상정지


시연 영상 : https://youtu.be/XqkPrCwqqmw
---

## 프로젝트 소개

반도체 패키지(IC칩 / 방열판 / 터미널블럭 / 커패시터)를 비전으로 분류·조립하는 5인 팀 프로젝트에서, 저는 **하드웨어와 펌웨어 전체**를 맡았습니다. 카메라가 부품을 잡으면 컨베이어가 흘려보내고, 게이트가 불량·중복을 쳐내고, 정상 트레이가 완성되면 AGV가 라인을 따라 운반한 뒤 자동으로 충전소로 들어가는 — 이 모든 물리적 흐름의 펌웨어 쪽을 담당했습니다.

본 저장소는 그중에서 **제가 직접 작성·튜닝한 두 개의 ESP32 펌웨어**(컨베이어계 + AGV 1·2호 공통)와 비상정지 이중 안전회로를 다룹니다. 비전 인식(YOLO + OpenCV)과 WPF HMI는 팀원이 담당했으며, 본 저장소에는 포함하지 않았습니다.

<img width="866" height="426" alt="image" src="https://github.com/user-attachments/assets/84963333-e3e6-4ac1-80b8-e93dc8a66dd7" />


---

## 전체 시스템에서 본인 담당 영역

```
                          ┌────────────────────────────┐
                          │ 팀원 담당 — Python 중앙서버  │
                          │  (YOLO 분류 + OpenCV 핀검사) │
                          │  + WPF HMI                  │
                          │  + myCobot 로봇팔 제어       │
                          └──────────┬─────────────────┘
                                     │ USB Serial (JSON)
                                     │ MQTT
            ┌────────────────────────┼────────────────────────┐
            │                                                  │
            ▼                                                  ▼
┌─────────────────────────┐                       ┌────────────────────────┐
│  ✅ 본인 담당              │                       │  ✅ 본인 담당             │
│  컨베이어 ESP32          │                       │  AGV 1·2호 ESP32       │
│  (Hardware-Connect)     │                       │  (공통 펌웨어)           │
├─────────────────────────┤                       ├────────────────────────┤
│ • 메인 컨베이어(스텝모터) │                       │ • QTR 8센서 라인트레이싱│
│ • 서브 컨베이어 ×2 (DC) │                       │ • PD 제어 (KP/KD 튜닝)  │
│ • 적외선 센서 → 트리거    │                       │ • RFID 노드 8개 인식    │
│ • 게이트 서보 ×2          │                       │ • 충전소 3개 빈자리 배정 │
│ • 거리 기반 푸셔 타이밍   │                       │ • 교대 운반 / 충전 복귀  │
│ • NC/NO 비상정지 이중화  │                       │ • HC-SR04 거리·MQTT 발행 │
└─────────────────────────┘                       └────────────────────────┘
            │                                                  │
            ▼                                                  ▼
   부품 분류 분배 라인                                    공정 간 운반
```

<img width="2000" height="1125" alt="image" src="https://github.com/user-attachments/assets/ed60ed3f-5b58-49db-a307-418f86fcab07" />


---

##  1. 컨베이어 ESP32 펌웨어

### 하드웨어 구성

| 구성 | 부품 | ESP32 핀 | 역할 |
|------|------|----------|------|
| 메인 컨베이어 | NEMA 스텝모터 + DM860A 드라이버 | PUL=14, DIR=12, ENBL=13 | 부품 이송 (1.72 cm/s, AccelStepper) |
| 중복 반환 컨베이어 | L9110S + DC 모터 (M1) | CONV2_A=26, CONV2_B=27 | 중복 검출 시 메인으로 순환 (상시 ON) |
| 트레이 공급 컨베이어 | L9110S + DC 모터 (M2) | CONV3_A=32, CONV3_B=33 | 빈 트레이 다음 칸 공급 (2초 자동 정지) |
| 게이트 1 (불량) | MG90S 서보 | GPIO 15 | 불량 부품 분기 (0°↔90°, 300ms 홀드) |
| 게이트 2 (중복) | MG90S 서보 | GPIO 2 | 중복 부품 분기 |
| 적외선 센서 | E18-D80NK | GPIO 34 | 카메라 촬영 트리거 (50ms 디바운스) |
| 비상정지 NC | 12V 전원선 직결 | (드라이버 전원) | DM860A 모터드라이버 직접 차단 |
| 비상정지 NO | 푸시버튼 | GPIO 25 (INPUT_PULLUP) | ESP32 인터럽트 발생 |

<img width="2000" height="1000" alt="최종 하드웨어 회로도1 ps" src="https://github.com/user-attachments/assets/bf24c397-5aa0-4506-9032-462d2ed00517" />

<img width="405" height="382" alt="image" src="https://github.com/user-attachments/assets/8c1d5ae6-f562-4b37-a906-63bd79a9e12f" />


### 핵심 — 거리 기반 게이트 타이밍

게이트 푸셔는 **단순 지연이 아니라 거리/속도 계산**으로 동작합니다. 적외선 센서가 부품을 감지하면 즉시 푸셔를 때리는 게 아니라, 게이트 위치까지 이동 시간을 계산해 정확한 순간에 푸셔가 나갑니다.

| 게이트 | 센서~게이트 거리 | 컨베이어 속도 | 계산된 푸셔 지연 |
|--------|-----------------|--------------|-----------------|
| Gate 1 (불량) | **30.2 cm** | 1.72 cm/s | ≈ 17.6 초 |
| Gate 2 (중복) | **39.8 cm** | 1.72 cm/s | ≈ 23.1 초 |

```cpp
unsigned long delay1 = (GATE1_DISTANCE / CONV_SPEED_CM) * 1000;
scheduleGate(1, delay1);
```

거리 값(`GATE1_DISTANCE`, `GATE2_DISTANCE`)과 속도(`CONV_SPEED_CM`)는 실측·튜닝으로 잡았고, 컨베이어 속도를 바꾸면 푸셔 지연이 자동으로 재계산됩니다. 실측 튜닝 도구(`gate_timing_tune.py`)는 팀원의 Python 서버에 있습니다.

###  NC/NO 이중 비상정지 — 산업안전 표준 결선

소프트웨어 한 가지만으로는 안전을 보장할 수 없어, 하드웨어(NC)와 소프트웨어(NO)를 **둘 다 동작하는 회로**로 분리했습니다.

| 채널 | 결선 | 차단 방식 | 효과 |
|------|------|----------|------|
| **NC** (Normally Closed) | 12V → 메인 컨베이어 DM860A 드라이버 전원선 | 물리 회로 차단 | **MCU와 무관하게 모터 전원 즉시 OFF** |
| **NO** (Normally Open) | 푸시버튼 → ESP32 GPIO25 인터럽트 | 펌웨어 인터럽트 | 게이트 IDLE 복귀 + 서브컨베이어 정지 + JSON 보고 |

```cpp
// 펌웨어 인터럽트 (NO 채널)
pinMode(ESTOP_PIN, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), onEstopPressed, FALLING);

void emergencyStop() {
  stopConveyor();
  gate1Servo.write(SERVO_IDLE);
  gate2Servo.write(SERVO_IDLE);
  conv2Stop();
  Serial.println("{\"type\":\"error\",\"status\":\"error\",\"message\":\"EMERGENCY STOP\"}");
}
```

**왜 둘 다 필요한가** — NC만 있으면 PC가 정지 사실을 모르고, NO만 있으면 ESP32가 죽거나 펌웨어가 멈추면 모터가 안 멈춥니다. 둘 중 하나가 실패해도 다른 하나가 살아 있어야 안전합니다.

또한 펌웨어에는 **5초 워치독 + 5초 시리얼 타임아웃**이 추가로 걸려 있어, 호스트 PC와 통신이 끊기거나 펌웨어가 멈춰도 자동으로 비상정지가 발동됩니다.

<img width="97" height="103" alt="image" src="https://github.com/user-attachments/assets/2e3728ac-20f2-43bd-8829-07c3e228da85" />


### 통신 프로토콜 — Python 서버 ↔ ESP32

USB Serial 115200 baud, **JSON 1라인 = 1명령** 방식. 각 명령에 정확히 1줄 JSON 응답.

| 방향 | 메시지 | 의미 |
|------|--------|------|
| PC → ESP32 | `{"type":"gate_cmd","gate":"1","action":"push"}` | 게이트 즉시 푸셔 |
| PC → ESP32 | `{"type":"conveyor_cmd","action":"set_speed","speed":1.5}` | 메인 컨베이어 속도 (cm/s, 0=정지) |
| PC → ESP32 | `{"type":"tray_cmd","action":"advance"}` | 트레이 공급 컨베이어 2초 ON |
| PC → ESP32 | `{"type":"emergency_stop"}` | 소프트 비상정지 |
| PC → ESP32 | `{"type":"ping"}` | 연결 확인 → `pong` |
| ESP32 → PC | `{"type":"sensor_triggered","timestamp":...}` | 적외선 센서 → 촬영 트리거 |
| ESP32 → PC | `{"type":"emergency_stop","source":"button"}` | 물리 버튼 눌림 보고 |
| ESP32 → PC | `{"type":"emergency_clear","source":"button"}` | 버튼 해제 보고 |

수동 시리얼 테스트용으로 텍스트 폴백(`GATE1:PUSH`, `STATUS` 등)도 유지했습니다.

---

##  2. AGV 1·2호 ESP32 펌웨어 (공통)

### 하드웨어 구성

| 구성 | 부품 | ESP32 핀 | 역할 |
|------|------|----------|------|
| 메인 보드 | ESP32-WROOM-32 (ESP32 Max 쉴드) | — | WiFi + MQTT 통신 |
| 4륜 모터 | DC 모터 ×4 (M1~M4) | ACB_SmartCar_V2 SDK | 4륜 독립 구동 |
| 라인 센서 | QTRX-MD-08RC (8채널 적외선) | D-pins | 라인트레이싱 + 교차로 감지 |
| RFID 리더 | MFRC522 | SPI (D5/D18/D19/D21/D22) | 노드 UID 인식 |
| 거리 센서 | HC-SR04 초음파 | Trig/Echo | 전방 장애물 감지 |
| 전원 | 18650 ×2 (7.4V) | Shield 전원 입력 | AGV 본체 전원 |




<img width="3027" height="1947" alt="최종 하드웨어 회로도 2 ps" src="https://github.com/user-attachments/assets/52d96ea1-d994-46a3-bc98-59c5d2f168e9" />
-AGV 회로도 


### RFID 노드 8종 — UID 기반 동작 매핑

라인 위 8개 RFID 태그를 UID로 식별해, 각 위치에서 정해진 동작(정지·회전·분기·창고 진입)을 수행합니다. UID는 `getNodeFromUid()`에서 분기되며, **인식 즉시 MQTT 토픽 `visipick/agv/1/rfid`로 발행**되어 WPF가 실시간 위치 맵핑에 사용합니다.

| UID | 노드 이름 | 동작 |
|-----|-----------|------|
| `D6:B9:39:F4` | START | 출발점 진입 — 정지 |
| `3B:47:43:5A` | START_TURN_STOP | 출발점 회전 완료 후 정지 |
| `3B:47:3D:5A` | HOME_JUNCTION | 충전소 분기점 |
| `3B:47:42:5A` | JUNCTION | 일반 교차로 |
| `E7:C0:3B:27` | SELECT_LEFT | 좌측 분기 선택 |
| `3B:47:44:5A` | SELECT_RIGHT | 우측 분기 선택 |
| `3B:47:45:5A` | WAREHOUSE_1 | 창고 1 진입 |
| `2A:7A:61:E1` | WAREHOUSE_2 | 창고 2 진입 |

같은 태그를 빠르게 두 번 읽는 것을 방지하기 위해 **2.5초 디바운스**(`RFID_SAME_UID_IGNORE_MS = 2500`)를 두었습니다.

<img width="372" height="413" alt="image" src="https://github.com/user-attachments/assets/8975f4c7-9a79-441e-89b3-93d12f270b24" />


### 라인트레이싱 — PD 제어

QTRX 8채널 센서의 위치값과 중앙(3500) 오차로 PD 제어합니다.

```cpp
int baseSpeed = 200;
float KP = 0.055;
float KD = 0.13;
```

`HIT_THRESHOLD=80`, `FLOOR_MARGIN=20`은 라인 검출 임계값으로, 바닥 반사율에 맞춰 실측 튜닝했습니다. KP·KD 값은 모터 응답·라인 폭에 맞춰 직접 조정한 값입니다.

### 자동 충전소 배정 (3대 슬롯)

3개의 충전 슬롯(`homeFree1/2/3`)을 관리하며, 충전이 필요한 AGV가 도착하면 **빈 슬롯을 자동으로 골라** 진입합니다.

```cpp
int selectFreeHome();        // 비어있는 충전 슬롯 번호 반환
void markSelectedHomeBusy(); // 선택한 슬롯을 사용 중으로 표시
bool tryStartHomeAfterFree();// 슬롯이 비기를 기다렸다가 출발
```

1호·2호가 같은 슬롯을 동시에 잡지 않도록 MQTT 상태 공유로 협조합니다.

### 교대 운반

완성된 트레이가 출발점에 오면, **충전소에서 대기하던 AGV가 먼저 출동**해 적재받고 출발합니다. 다른 AGV는 운반을 마치고 돌아오면 자기 자리에서 충전을 시작 — 두 대가 번갈아 운반하며 라인이 멈추지 않게 합니다.

### MQTT 상태 발행

AGV의 실시간 상태(현재 노드, 미션, 다음 동작, RFID UID, 거리, 트레이 적재 여부 등)를 주기적으로 발행합니다.

```json
{
  "agv_id": "AGV_1",
  "mission": "...",
  "node_id": 0,
  "node": "...",
  "next_action": "...",
  "rfid_uid": "...",
  "dist_cm": -1
}
```

WPF HMI는 이 데이터를 받아 라인 위에 AGV 위치를 실시간으로 그립니다.

<img width="724" height="305" alt="image" src="https://github.com/user-attachments/assets/6d0228b1-9835-4eb0-90a7-f6db26f10897" />



---

## 🛠 기술 스택

| 영역 | 사용 기술 |
|------|----------|
| MCU | ESP32-WROOM-32 ×3 (컨베이어 1대 + AGV 2대) |
| 펌웨어 | Arduino C++, FreeRTOS task watchdog |
| 라이브러리 | ESP32Servo, AccelStepper, ArduinoJson v7, QTRSensors, MFRC522, PubSubClient (MQTT), ACB_SmartCar_V2 |
| 모터 드라이버 | DM860A (스텝모터), L9110S (DC 모터), ACB_SmartCar 4륜 드라이버 |
| 통신 | USB Serial (JSON), WiFi + MQTT |
| 안전 | NC 하드웨어 차단 + NO 인터럽트 + 워치독 5s + 시리얼 타임아웃 5s |

---

##  폴더 구조

```
.
├── conveyor-firmware/
│   └── esp32.ino              # 컨베이어 ESP32 펌웨어 (603줄)
│
├── agv-firmware/
│   └── agv1.ino               # AGV 1·2호 공통 펌웨어 (2,482줄)
│                              # ※ AGV ID는 코드 상단 상수로 분기
│
├── circuit/
│   ├── conveyor_circuit.png   # 컨베이어 회로도 (Fritzing)
│   └── agv_circuit.png        # AGV 회로도 (Fritzing)
│
└── docs/
    └── rfid_node_map.md       # RFID 노드 UID 매핑 표 (별도 문서)
```

---

##  빌드 / 업로드

### 컨베이어 ESP32

1. Arduino IDE에서 ESP32 보드 추가
2. 라이브러리 매니저로 설치: `ESP32Servo`, `AccelStepper`, `ArduinoJson` (v7 이상)
3. 보드: **ESP32 Dev Module**, 업로드 속도 921600
4. `conveyor-firmware/esp32.ino` 업로드
5. 시리얼 모니터 115200으로 `{"type":"ping"}` 입력 → `pong` 응답 확인

### AGV ESP32

1. 라이브러리 설치: `QTRSensors`, `MFRC522`, `PubSubClient`, `WiFi`, `ACB_SmartCar_V2`
2. 코드 상단의 WiFi SSID·PW, MQTT 브로커 IP 입력
3. 1호·2호 구분이 필요한 경우 `TOPIC_RFID_UID = "visipick/agv/1/rfid"` → `"agv/2/rfid"`로 수정
4. ESP32 Max 쉴드에 18650 ×2 (7.4V) 장착 후 업로드

---

##  팀 구성

| 담당 | 영역 |
|------|------|
| **본인 (하드웨어·펌웨어 전체)** | 컨베이어 ESP32 펌웨어 / AGV ESP32 펌웨어 / 라인 설계 / RFID 노드 아이디어·매핑 / NC/NO 비상정지 회로 / 회로도 작성 / 게이트 타이밍 튜닝 / 라인트레이싱 PD 튜닝 (보조: 로봇팔 티칭, YOLO 데이터 촬영) |
| 팀원 4명 (별도 저장소) | Python 중앙서버 (YOLO 분류, OpenCV 핀검사, FSM 오케스트레이션, FastAPI, MQTT 브리지) / WPF HMI / myCobot 로봇팔 제어 |

> 본 저장소는 **본인이 작성한 펌웨어·회로**만 포함합니다. 비전·서버·HMI 전체 시스템은 팀의 별도 저장소에서 관리됩니다.

---

##  한계 및 개선 방향

**WiFi/MQTT 끊김 시 AGV 복구 정책**
현재 AGV 펌웨어는 MQTT 연결이 끊겨도 마지막 명령으로 계속 주행합니다. 연결 끊김 감지 후 안전한 위치까지 천천히 진행 후 정지하는 fallback이 필요합니다.

**컨베이어 속도 변경 시 in-flight 부품 처리**
컨베이어 속도가 바뀌면 그 이후 감지되는 부품에는 새 속도 기반 푸셔 지연이 적용되지만, **이미 라인 위를 이동 중인 부품**의 푸셔 지연은 갱신되지 않습니다. 속도 변경을 막거나, 이미 스케줄된 푸셔의 잔여 거리를 재계산하는 로직이 필요합니다.

**RFID UID 하드코딩**
8개 노드 UID가 펌웨어 상수로 박혀 있어, 새 라인 추가 시 펌웨어 재업로드가 필요합니다. 향후 SPIFFS/Preferences로 분리해 무선 갱신이 가능하게 하면 좋습니다.

**충전소 슬롯 동기화**
`homeFree1/2/3`은 각 AGV가 MQTT로 공유하지만, 두 AGV가 동시에 같은 슬롯을 잡으려는 레이스를 완전히 막지는 못합니다. 서버 측 슬롯 락(중앙서버에서 토큰 발급)이 더 안전합니다.

---

> 본 저장소는 학습 및 포트폴리오 목적으로 제작되었습니다.
> 비전·HMI·로봇 제어 전체 시스템은 5인 팀 프로젝트의 일부이며, 본 문서는 본인이 담당한 영역(하드웨어/펌웨어/회로/라인 설계)을 다룹니다.
