/* ============================================================
   T870_FIELD_v12_0905.ino
   T870 현장 시험 도구 — 팀 배포판

   v11 변경 (v10 기반)
     ★ 구동을 5 PWM 단위 증감으로   w = 전진 +5, b = 후진 +5, e = 현재방향 -5
        누를 때마다 즉시 인가. 램프 없음. 상한은 255 에서만 막는다.
     ★ 조향각을 ROS 규약(+ = 우측)으로도 같이 표시
        이 차는 ADC 가 커지면 좌회전이다. ROS 토픽(/mcu/cmd_wheel)은 + 가 우측이다.
        부호가 반대라 헷갈리므로 두 표기를 나란히 찍는다.
          deg_ros = (centerAdc - adc) / COUNTS_PER_DEG      ← + 가 우측
          ADC 증가 = 좌회전 = deg_ros 감소

   v10 변경 (v9 기반, 실측값은 v9 것을 그대로 유지)
     ★ E-Stop 추가        하드웨어 D24 감시 + E 로 소프트 걸기 + R 로 해제
     ★ 5초 주행 시험 추가  2  (10초는 기존대로 1)
     ★ ADC 카운트 단위 미세 이동
          q / w    좌 / 우  1 카운트
          s / f    좌 / 우  5 카운트
          z / x    좌 / 우 10 카운트
       (기존 L/R 은 "도" 단위, 이건 "ADC 카운트" 단위. 중앙을 정밀하게 찾을 때 쓴다)
     ★ 정지키에 스페이스 추가 (0 / x 는... x 가 좌10 으로 갔으므로 0 과 스페이스)

   ⚠ v9 의 실측값·하드리밋·핀은 하나도 건드리지 않았다.

   무엇을 하는가
     (1) 조향을 1도 단위로 지정한다        L1~L22 / R1~R22 / C
     (2) 조향 ADC 를 직접 지정한다         G<숫자>
     (3) 10초 주행 시험으로 엔코더를 측정한다   1
     (4) 이동거리를 입력하면 counts_per_meter 를 계산한다   M<미터>

   v9 변경
     ★ 주행 시험을 5초 -> 10초
     ★ 부팅 시 자동 중앙정렬(호밍) 제거
        켜자마자 바퀴가 움직이면 헷갈리고 위험하다.
        중앙으로 보내고 싶으면 C 를 직접 누른다.
     ★ Z(호밍 재실행) 제거

   보드   : Arduino Mega 2560
   Baud   : 115200

   0904 실측값 (이 스케치의 근거)
     조향 중앙 ADC        355
     1도당 ADC 카운트     14.5   (320카운트 = 22도)
     최대 조향각          ±22도  (ADC 36 ~ 674)
     조향 방향            ADC 가 커지면 좌회전
     애커만               좌회전 시 오른쪽 23도 / 왼쪽 21도
                          (측정 오차 아님. 이 차종 특성)

   핀 (T870_MCU_v37.ino 217~247줄과 동일)
     PWM_DRIVE_FRONT=9  DIR_DRIVE_FRONT=10  FWD=LOW
     PWM_DRIVE_REAR =7  DIR_DRIVE_REAR =8   FWD=HIGH   (앞뒤 반대!)
     PWM_STEER=11  DIR_STEER=12  STEER_PWM_LEVEL=130
     ENC_A=2  ENC_B=3   조향각 A0

   ⚠ 주행 명령은 램프가 없다. 누르는 즉시 PWM 이 걸린다.
   ⚠ 조향 하드리밋 25~700 을 벗어나면 즉시 정지한다.
      포텐셔미터가 이 밖으로 나가면 내부가 갈린다.

   v2 변경
     (1) 조향 프리셋  L1~L11 / R1~R11 (v7 에서 11단 2도로 재분할)
     (2) 부팅 시 자동 중앙 복귀
     (3) S 명령이 그 시점 전체 로그를 출력
     (4) 안전 한계 100~860

   유지
     - 구동 PWM 램프 없음. 명령 즉시 앞/뒤 동시 인가
     - counts_per_meter 실측 (o 리셋 -> 주행 -> M<미터>)
     - 조향 폐루프 이동 (G<n>), 미세 조정 (a/d), 중앙 기록 (K)

   보드   : Arduino Mega 2560
   Baud   : 115200

   핀 (T870_MCU_v37.ino 217~247줄)
     PWM_DRIVE_FRONT=9  DIR_DRIVE_FRONT=10  FWD=LOW
     PWM_DRIVE_REAR =7  DIR_DRIVE_REAR =8   FWD=HIGH   (앞뒤 반대!)
     PWM_STEER=11  DIR_STEER=12  STEER_PWM_LEVEL=130
     ENC_A=2  ENC_B=3   조향각 A0

   0904 실측 (기어 재조정 후)
     기구 중앙 355 (재조정 후) / 사용 범위 36~674 (±22도) / 하드리밋 25~700
     ⚠ 이 범위를 넘기면 포텐셔미터 내부가 갈린다. 하드 리밋으로 막아둠.

   ⚠ 부팅하면 조향이 자동으로 움직인다. 사람/장애물 치워두고 켤 것.
   ============================================================ */

// ---------- 핀 ----------
constexpr uint8_t PIN_POT          = A0;
constexpr uint8_t PWM_STEER        = 11;
constexpr uint8_t DIR_STEER        = 12;

constexpr uint8_t PWM_DRIVE_FRONT  = 9;
constexpr uint8_t DIR_DRIVE_FRONT  = 10;
constexpr uint8_t PWM_DRIVE_REAR   = 7;
constexpr uint8_t DIR_DRIVE_REAR   = 8;

constexpr uint8_t ENC_A = 2;
constexpr uint8_t ENC_B = 3;

// ---------- 극성 (v37 244~247줄) ----------
constexpr uint8_t DRIVE_FRONT_FWD = LOW;
constexpr uint8_t DRIVE_FRONT_REV = HIGH;
constexpr uint8_t DRIVE_REAR_FWD  = HIGH;
constexpr uint8_t DRIVE_REAR_REV  = LOW;

// ---------- 설정 ----------
constexpr uint32_t BAUD = 115200;

// 🔴 v37 과 동일. 절대 바꾸지 말 것.
constexpr uint32_t ENCODER_DEBOUNCE_US = 200;

// ---- v10: E-Stop (T870_MCU_v37.ino 234줄과 동일 배선) ----
//   D24, INPUT_PULLUP, HIGH = 눌림. NC 접점이라 단선도 정지로 처리된다.
constexpr uint8_t  ESTOP_PIN          = 24;
constexpr uint32_t ESTOP_DEBOUNCE_MS  = 30;
bool     estopActive      = false;
bool     estopLatched     = false;   // 한 번 걸리면 R 로 풀 때까지 유지
bool     lastEstopReading = false;
uint32_t estopChangeMs    = 0;

// ---- 조향 5단 프리셋 ----
// ---- 조향 프리셋 (0904 실측 기반) ----
//   중앙 355 / 320카운트 = 22도 → 1도당 14.5카운트
//   1단 = 1도, 22단 = 22도 = 319카운트
constexpr uint16_t STEER_CENTER_DEFAULT = 355;    // 0904 기구 중앙 실측
constexpr float    COUNTS_PER_DEG       = 14.5f;  // 1도당 ADC 카운트
constexpr uint8_t  STEER_STAGES         = 22;     // 22단 -> 최대 ±22도
constexpr float    DEG_PER_STAGE        = 1.0f;   // 1단 = 1도

// ★ ADC 가 커지는 쪽이 좌회전이다 (0904 실측으로 확인)
//   -1 이면 L(음수 stage) 일 때 ADC 가 증가한다.
constexpr int8_t   STEER_DIR_SIGN       = -1;

// ---- 하드 리밋 (절대 넘지 않는다) ----
// 사용 범위 35~675 (중앙 355 ± 320) 바깥으로 살짝 여유를 둔 값.
// 왼쪽을 25 까지만 여는 이유: ADC 가 0 에 닿으면 포화라 값이 죽는다.
constexpr uint16_t ADC_HARD_LO = 25;
constexpr uint16_t ADC_HARD_HI = 700;

constexpr uint8_t  STEER_PWM_LEVEL = 130;
constexpr uint8_t  TOLERANCE       = 3;
constexpr uint8_t  PULSE_MIN_MS    = 5;
constexpr uint8_t  PULSE_MAX_MS    = 40;
constexpr uint8_t  ERR_PER_MS      = 8;
constexpr uint16_t SETTLE_MS       = 60;
constexpr uint32_t MOVE_TIMEOUT_MS = 4000;
constexpr uint8_t  STALL_LIMIT     = 20;
constexpr uint8_t  ADC_SAMPLES     = 8;

constexpr uint8_t  PWM_STEP    = 5;    // ★ v11: w/b/e 한 번에 바뀌는 양
constexpr uint8_t  PWM_STAGE_1 = 50;
constexpr uint8_t  PWM_STAGE_2 = 100;
constexpr uint8_t  PWM_STAGE_3 = 150;

constexpr uint32_t AUTO_STOP_MS = 30000;
constexpr uint32_t PRINT_MS     = 250;


// ---------- 엔코더 (ISR) ----------
volatile uint32_t countA = 0;
volatile uint32_t countB = 0;
volatile uint32_t lastAUs = 0;
volatile uint32_t lastBUs = 0;

// ---------- 조향 상태 ----------
uint16_t centerAdc  = STEER_CENTER_DEFAULT;
uint16_t target     = STEER_CENTER_DEFAULT;
int8_t   curStage   = 0;         // -5 ~ +5 (음수=L, 양수=R, 0=중앙)
bool     dirInvert  = false;
bool     dirLearned = false;
bool     moving     = false;
uint32_t moveStart  = 0;
uint8_t  stallCount = 0;
uint16_t pulseCount = 0;
uint8_t  nudgeStep  = 5;

// 호밍 시퀀스: 0=대기 1=L2로 2=R2로 3=중앙으로 4=완료
// ---- 10초 주행 시험 ----
constexpr uint32_t TEST_RUN_MS    = 10000;  // 1 = 정확히 10.00초
constexpr uint32_t TEST_RUN_MS_5S = 5000;   // v10: 2 = 정확히 5.00초
uint32_t testRunMs = TEST_RUN_MS;           // v10: 이번 시험의 길이
constexpr uint8_t  TEST_RUN_PWM   = 50;
constexpr uint32_t TEST_COAST_MS  = 3000;   // 코스팅이 끝날 때까지 더 세는 시간

uint8_t  testPhase   = 0;    // 0=대기 1=주행중 2=코스팅중
uint32_t testStartMs = 0;
uint32_t testStopMs  = 0;
uint32_t testCountAtStop = 0;
uint32_t testElapsedMs   = 0;

char     lastResult[24] = "NONE";

// ---------- 주행 상태 ----------
uint8_t  drivePwm   = 0;
bool     dirForward = true;
uint32_t cmdTime    = 0;
bool     running    = false;

uint32_t tPrint = 0;
uint32_t prevA = 0, lastRateMs = 0, rateA = 0;

// ---------- 명령 버퍼 (전역: waitOrAbort 도 참조한다) ----------
char    cmdBuf[16];
uint8_t cmdLen = 0;

// 정지 키인가? 명령 첫 글자일 때만 정지로 본다.
// (0 은 G100 / M5 등 명령 안에도 들어가므로 반드시 이 조건이 필요하다)
inline bool isStopChar(char c) {
  //  v10: 스페이스도 정지로 받는다. 급할 때 아무거나 눌러도 서게.
  return (cmdLen == 0) && (c == '0' || c == 'x' || c == 'X' || c == ' ');
}

// ---------- ISR ----------
void isrA() {
  uint32_t now = micros();
  if (now - lastAUs >= ENCODER_DEBOUNCE_US) { lastAUs = now; countA++; }
}
void isrB() {
  uint32_t now = micros();
  if (now - lastBUs >= ENCODER_DEBOUNCE_US) { lastBUs = now; countB++; }
}

// ---------- 저수준 ----------
uint16_t readAdc() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) { s += analogRead(PIN_POT); delayMicroseconds(200); }
  return (uint16_t)(s / ADC_SAMPLES);
}

void steerOff() { analogWrite(PWM_STEER, 0); }

void applyDriveDir() {
  digitalWrite(DIR_DRIVE_FRONT, dirForward ? DRIVE_FRONT_FWD : DRIVE_FRONT_REV);
  digitalWrite(DIR_DRIVE_REAR,  dirForward ? DRIVE_REAR_FWD  : DRIVE_REAR_REV);
}

// ★ 램프 없음. 앞/뒤 동시에 즉시 인가.
void applyDrivePwm(uint8_t v) {
  analogWrite(PWM_DRIVE_FRONT, v);
  analogWrite(PWM_DRIVE_REAR,  v);
}

void driveStop(const __FlashStringHelper* why) {
  drivePwm = 0;
  applyDrivePwm(0);
  running = false;
  Serial.print(F("DRIVE_STOP,")); Serial.println(why);
}

//  ★ v11 — 5 PWM 씩 올리고 내린다.
//
//    delta > 0 : 그 방향으로 PWM 을 5 올린다 (255 에서 멈춘다)
//    delta < 0 : 지금 방향의 PWM 을 5 내린다 (0 이 되면 정지)
//    램프는 없다. 계산한 값을 그 자리에서 analogWrite 한다.
void driveStep(bool forward, int delta) {
  if (estopLatched) { Serial.println(F("ERR,E-Stop 래치.  ~ 로 해제")); return; }

  int cur = (int)drivePwm;
  //  방향이 바뀌면 지금 값은 버리고 0 부터 다시 올린다.
  //  (돌던 방향 그대로 반대로 꽂으면 역토크가 크다)
  if (running && forward != dirForward) {
    applyDrivePwm(0);
    delay(300);
    cur = 0;
  }

  int next = cur + delta;
  if (next < 0)   next = 0;
  if (next > 255) next = 255;

  dirForward = forward;
  applyDriveDir();
  drivePwm = (uint8_t)next;
  applyDrivePwm(drivePwm);          // 즉시 인가
  running  = (drivePwm > 0);
  cmdTime  = millis();

  Serial.print(F("DRIVE,"));
  Serial.print(forward ? F("FWD") : F("REV"));
  Serial.print(F(",PWM ")); Serial.print(cur);
  Serial.print(F(" -> "));   Serial.print(drivePwm);
  if (drivePwm == 255) Serial.print(F("  (상한)"));
  if (drivePwm == 0)   Serial.print(F("  (정지)"));
  Serial.println();
}

void setDrive(bool forward, uint8_t pwm) {
  if (estopLatched) { Serial.println(F("ERR,E-Stop 래치. R 로 해제")); return; }
  if (running && forward != dirForward) {
    applyDrivePwm(0);
    delay(300);
  }
  dirForward = forward;
  applyDriveDir();
  drivePwm = pwm;
  applyDrivePwm(pwm);
  cmdTime = millis();
  running = true;
  Serial.print(F("DRIVE,")); Serial.print(forward ? F("FWD") : F("REV"));
  Serial.print(F(",pwm=")); Serial.print(pwm);
  Serial.println(F(" (즉시)"));
}

void resetCounts() {
  noInterrupts(); countA = 0; countB = 0; interrupts();
  prevA = 0; rateA = 0; lastRateMs = millis();
  Serial.println(F("RESET,countA=0,countB=0"));
}

// ---------- 프리셋 계산 ----------
// stage: -22 ~ +22.  음수 = L(좌회전), 양수 = R(우회전).  1단 = 1도.
// 1도가 14.5카운트라 정수로 안 떨어진다 → 실수로 곱한 뒤 반올림한다.
//  ★ v11 — ROS 규약 조향각.  + 가 우측 (/mcu/cmd_wheel 과 같은 부호)
//
//    이 차는 ADC 가 커지면 **좌회전** 이다 (0904 실측).
//    ROS 토픽은 + 가 **우측** 이다. 부호가 반대라 반드시 뒤집어야 한다.
//    여기서 한 번만 뒤집고, 화면에는 두 표기를 나란히 찍는다.
float adcToDegRos(int32_t adc) {
  return ((float)centerAdc - (float)adc) / COUNTS_PER_DEG;
}

int32_t stageToAdc(int8_t stage) {
  float off = (float)stage * COUNTS_PER_DEG;
  int32_t o = (int32_t)(off >= 0 ? off + 0.5f : off - 0.5f);
  return (int32_t)centerAdc + (int32_t)STEER_DIR_SIGN * o;
}

// ---------- v10: E-Stop ----------
//   v37 과 같은 배선/극성. NC 접점이라 단선도 정지로 처리된다.
//   한 번 걸리면 래치되어 R 로 풀기 전까지 아무것도 못 움직인다.
void estopStopEverything() {
  applyDrivePwm(0);
  drivePwm = 0;
  running  = false;
  analogWrite(PWM_STEER, 0);
  moving   = false;
  testPhase = 0;
}

void updateEstop() {
  uint32_t now = millis();
  bool reading = (digitalRead(ESTOP_PIN) == HIGH);

  if (reading != lastEstopReading) {
    estopChangeMs    = now;
    lastEstopReading = reading;
    return;
  }
  if (now - estopChangeMs < ESTOP_DEBOUNCE_MS) return;
  if (reading == estopActive) return;

  estopActive = reading;
  if (estopActive) {
    estopLatched = true;
    estopStopEverything();
    Serial.println();
    Serial.println(F("*** E-STOP *** 전부 정지. 래치됨"));
    Serial.println(F("  버튼을 풀고  R  을 눌러야 다시 움직인다"));
  } else {
    Serial.println(F("E-Stop 버튼 해제됨 (래치는 아직. R 로 해제)"));
  }
}

// ---------- 10초 주행 시험 ----------
void startTestRun(uint32_t runMs) {
  if (estopLatched) { Serial.println(F("ERR,E-Stop 래치. R 로 해제")); return; }
  if (testPhase != 0) {
    Serial.println(F("ERR,시험이 이미 진행 중이다. 0 으로 중단해라"));
    return;
  }
  Serial.println();
  Serial.println(F("======== 주행 시험 시작 ========"));
  Serial.print  (F("PWM ")); Serial.print(TEST_RUN_PWM);
  testRunMs = runMs;                              // v10
  Serial.print  (F(" 전진, ")); Serial.print(testRunMs / 1000.0f, 2);
  Serial.println(F("초"));
  Serial.println(F("출발점을 바닥에 표시해 두어라"));
  Serial.println(F("중단하려면 0"));

  resetCounts();
  dirForward = true;
  applyDriveDir();
  drivePwm = TEST_RUN_PWM;
  applyDrivePwm(TEST_RUN_PWM);      // 램프 없음. 즉시 인가
  running = true;
  cmdTime = millis();
  testStartMs = millis();
  testPhase = 1;
}

void abortTestRun() {
  if (testPhase != 0) {
    testPhase = 0;
    Serial.println(F("TEST_RUN,중단됨"));
  }
}

void updateTestRun() {
  if (testPhase == 0) return;

  if (testPhase == 1) {
    if (millis() - testStartMs >= testRunMs) {
      applyDrivePwm(0);
      drivePwm = 0;
      running = false;
      testStopMs = millis();
      testElapsedMs = testStopMs - testStartMs;
      noInterrupts(); testCountAtStop = countA; interrupts();

      Serial.println();
      Serial.print(F("TEST_RUN,모터 정지  경과 "));
      Serial.print(testElapsedMs / 1000.0f, 2);
      Serial.println(F("초"));
      Serial.print(F("  구동 구간 카운트 : ")); Serial.println(testCountAtStop);
      Serial.print(F("  코스팅 대기 "));
      Serial.print(TEST_COAST_MS / 1000);
      Serial.println(F("초..."));
      testPhase = 2;
    }
    return;
  }

  if (testPhase == 2) {
    if (millis() - testStopMs >= TEST_COAST_MS) {
      noInterrupts(); uint32_t fin = countA; interrupts();

      Serial.println();
      Serial.println(F("========== 주행 시험 결과 =========="));
      Serial.print(F("주행 시간        : "));
      Serial.print(testElapsedMs / 1000.0f, 2); Serial.println(F(" 초"));
      Serial.print(F("PWM              : ")); Serial.println(TEST_RUN_PWM);
      Serial.print(F("구동 구간 카운트 : ")); Serial.println(testCountAtStop);
      Serial.print(F("코스팅 포함 최종 : ")); Serial.println(fin);
      Serial.print(F("코스팅 카운트    : ")); Serial.println(fin - testCountAtStop);
      if (fin > 0) {
        Serial.print(F("코스팅 비율      : "));
        Serial.print((float)(fin - testCountAtStop) * 100.0f / (float)fin, 1);
        Serial.println(F(" %"));
      }
      if (testElapsedMs > 0) {
        Serial.print(F("평균 카운트/초   : "));
        Serial.println((float)testCountAtStop * 1000.0f / (float)testElapsedMs, 1);
      }
      Serial.println(F("-----------------------------------"));
      Serial.println(F("줄자로 이동거리를 재라 (출발 표시 ~ 최종 정지점)"));
      Serial.println(F("그 값을 M<미터> 로 입력하면 counts_per_meter 가 나온다"));
      Serial.println(F("  예:  M4.72"));
      Serial.println(F("==================================="));
      Serial.println();
      testPhase = 0;
    }
  }
}

// ---------- 거리 계산 ----------
void printCpm(float meters) {
  noInterrupts(); uint32_t a = countA, b = countB; interrupts();

  Serial.println();
  Serial.println(F("======== counts_per_meter 실측 ========"));
  Serial.print(F("주행거리      : ")); Serial.print(meters, 2); Serial.println(F(" m"));
  Serial.print(F("countA        : ")); Serial.println(a);
  Serial.print(F("countB        : ")); Serial.println(b);
  if (b > 0) {
    Serial.print(F("A/B 비율      : ")); Serial.println((float)a / (float)b, 3);
    Serial.println(F("  (1.0 근처면 B상 정상 -> 쿼드러처 가능성)"));
  } else {
    Serial.println(F("  B상 0 -> A상 단독 유지"));
  }
  if (meters > 0.01f && a > 0) {
    Serial.print(F("counts_per_m  : ")); Serial.println((float)a / meters, 1);
    Serial.print(F("1카운트 거리  : "));
    Serial.print((meters * 1000.0f) / (float)a, 2); Serial.println(F(" mm"));
  }
  Serial.println(F("펌웨어 현재값 : 199.8 (0829 실측)"));
  Serial.println(F("======================================"));
  Serial.println();
}

// ---------- 조향 폐루프 ----------
bool waitOrAbort(uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial.available()) {
      char c = (char)Serial.peek();
      if (isStopChar(c)) return false;
    }
  }
  return true;
}

void stopMove(const char* reason) {
  steerOff();
  moving = false;
  strncpy(lastResult, reason, sizeof(lastResult) - 1);
  lastResult[sizeof(lastResult) - 1] = '\0';

  uint16_t adc = readAdc();
  Serial.print(F("STEER_STOP,")); Serial.print(reason);
  Serial.print(F(",adc=")); Serial.print(adc);
  Serial.print(F(",target=")); Serial.print(target);
  Serial.print(F(",err=")); Serial.print((int16_t)target - (int16_t)adc);
  Serial.print(F(",pulses=")); Serial.println(pulseCount);
}

void startMove(int32_t t, int8_t stage) {
  if (estopLatched) { Serial.println(F("ERR,E-Stop 래치. R 로 해제")); return; }
  if (t < (int32_t)ADC_HARD_LO || t > (int32_t)ADC_HARD_HI) {
    Serial.print(F("ERR,목표 ")); Serial.print(t);
    Serial.print(F(" 는 하드리밋 밖 ("));
    Serial.print(ADC_HARD_LO); Serial.print(F("~")); Serial.print(ADC_HARD_HI);
    Serial.println(F("). 무시함"));
    return;
  }
  target = (uint16_t)t;
  curStage = stage;
  moving = true;
  moveStart = millis();
  stallCount = 0;
  pulseCount = 0;
  dirLearned = false;

  Serial.print(F("STEER_GOTO,from=")); Serial.print(readAdc());
  Serial.print(F(",to=")); Serial.print(target);
  Serial.print(F(",stage="));
  if (stage == 0) Serial.println(F("C"));
  else { Serial.print(stage < 0 ? 'L' : 'R'); Serial.println(stage < 0 ? -stage : stage); }
}

void stepMove() {
  uint16_t adc = readAdc();
  int16_t  err = (int16_t)target - (int16_t)adc;

  if (adc < ADC_HARD_LO || adc > ADC_HARD_HI) { stopMove("HARD_LIMIT"); return; }
  if (err <= TOLERANCE && err >= -TOLERANCE)  { stopMove("REACHED");    return; }
  if (millis() - moveStart > MOVE_TIMEOUT_MS) { stopMove("TIMEOUT");    return; }

  int16_t a = err < 0 ? -err : err;
  uint16_t p = a / ERR_PER_MS;
  if (p < PULSE_MIN_MS) p = PULSE_MIN_MS;
  if (p > PULSE_MAX_MS) p = PULSE_MAX_MS;

  bool wantUp = (err > 0);
  bool level  = dirInvert ? !wantUp : wantUp;

  digitalWrite(DIR_STEER, level ? HIGH : LOW);
  analogWrite(PWM_STEER, STEER_PWM_LEVEL);
  bool ok = waitOrAbort(p);
  steerOff();
  pulseCount++;
  if (!ok) { stopMove("USER_ABORT"); return; }
  if (!waitOrAbort(SETTLE_MS)) { stopMove("USER_ABORT"); return; }

  uint16_t adc2 = readAdc();
  int16_t delta = (int16_t)adc2 - (int16_t)adc;

  if (!dirLearned) {
    int16_t ad = delta < 0 ? -delta : delta;
    if (ad >= 3) {
      if ((delta > 0) != wantUp) {
        dirInvert = !dirInvert;
        Serial.print(F("DIR_FLIP,invert=")); Serial.println(dirInvert ? 1 : 0);
      }
      dirLearned = true;
    }
  }

  int16_t ad2 = delta < 0 ? -delta : delta;
  if (ad2 <= 1) {
    if (++stallCount >= STALL_LIMIT) { stopMove("STALL"); return; }
  } else stallCount = 0;

  Serial.print(F("STEER,adc=")); Serial.print(adc2);
  Serial.print(F(",err=")); Serial.print((int16_t)target - (int16_t)adc2);
  Serial.print(F(",d=")); Serial.println(delta);
}

// ---------- 도움말 ----------
void printHelp() {
  Serial.println(F("========== 조향  (전부 ADC 폐루프. ms 로 미는 코드 없음) =========="));
  Serial.print  (F("  중앙 ")); Serial.print(centerAdc);
  Serial.print  (F("  /  1도 = ")); Serial.print(COUNTS_PER_DEG, 1);
  Serial.print  (F(" 카운트  /  하드리밋 "));
  Serial.print(ADC_HARD_LO); Serial.print(F("~")); Serial.println(ADC_HARD_HI);
  Serial.println(F("  ※ ADC 가 커지면 좌회전. ROS 토픽은 + 가 우측이라 부호가 반대다"));
  Serial.println();
  Serial.println(F("  G<n>      ADC n 으로 이동        예) G400"));
  Serial.println(F("  C         기록된 중앙으로        (K 로 기록한 값)"));
  Serial.println(F("  J         기준 중앙 355 로       (K 와 무관. 되돌리기용)"));
  Serial.print  (F("  L1~L")); Serial.print(STEER_STAGES);
  Serial.println(F("    왼쪽  n도  (ADC 증가)"));
  Serial.print  (F("  R1~R")); Serial.print(STEER_STAGES);
  Serial.println(F("    오른쪽 n도  (ADC 감소)"));
  Serial.println(F("  , / .     좌 / 우  1 카운트   (약 0.07도)"));
  Serial.println(F("  < / >     좌 / 우  5 카운트"));
  Serial.println(F("  [ / ]     좌 / 우 10 카운트"));
  Serial.println(F("  a / d     좌 / 우  N 카운트   (N<n> 으로 폭 변경)"));
  Serial.println(F("  K         ★ 지금 위치를 중앙으로 기록"));
  Serial.println(F("  I         조향 모터 방향 반전"));
  Serial.println();
  Serial.println(F("========== 구동  (램프 없음. 누르는 즉시 인가) =========="));
  Serial.println(F("  w         전진 PWM +5"));
  Serial.println(F("  b         후진 PWM +5"));
  Serial.println(F("  e         지금 방향 PWM -5   (0 이 되면 정지)"));
  Serial.println(F("            상한 255. 방향을 바꾸면 0 부터 다시 올라간다"));
  Serial.println();
  Serial.println(F("========== 주행 시험  (엔코더 자동 측정) =========="));
  Serial.println(F("  1         10초 주행  (PWM 50)"));
  Serial.println(F("  2          5초 주행  (PWM 50)"));
  Serial.println(F("            카운터 자동리셋 -> 주행 -> 코스팅 3초 -> 결과"));
  Serial.println(F("  o         엔코더 카운터만 리셋"));
  Serial.println(F("  M<n>      n미터 갔다 -> counts_per_meter   예) M4.85"));
  Serial.println();
  Serial.println(F("========== 정지 · 안전 =========="));
  Serial.println(F("  0 / x / 스페이스   즉시 정지"));
  Serial.println(F("  !         E-Stop 걸기 (래치)"));
  Serial.println(F("  ~         E-Stop 래치 해제"));
  Serial.println(F("            하드웨어 E-Stop(D24) 은 항상 감시한다"));
  Serial.println(F("            주행 30초 자동정지"));
  Serial.println();
  Serial.println(F("========== 기타 =========="));
  Serial.println(F("  S         현재 전체 상태"));
  Serial.println(F("  N<n>      a/d 이동 폭 변경"));
  Serial.println(F("  h         이 도움말"));
  Serial.println(F("----------------------------------------"));
}

// ---------- S: 전체 로그 ----------
void printFullLog() {
  noInterrupts(); uint32_t a = countA, b = countB; interrupts();
  uint16_t adc = readAdc();

  Serial.println();
  Serial.println(F("=========== 현재 상태 전체 ==========="));
  Serial.print(F("가동시간      : ")); Serial.print(millis() / 1000); Serial.println(F(" s"));
  Serial.println(F("--- 조향 ---"));
  Serial.print(F("현재 ADC      : ")); Serial.println(adc);
  Serial.print(F("중앙 ADC      : ")); Serial.println(centerAdc);
  Serial.print(F("목표 ADC      : ")); Serial.println(target);
  Serial.print(F("오차          : ")); Serial.println((int16_t)target - (int16_t)adc);
  Serial.print(F("현재 단수     : "));
  if (curStage == 0) Serial.println(F("C (중앙)"));
  else { Serial.print(curStage < 0 ? 'L' : 'R');
         Serial.print(curStage < 0 ? -curStage : curStage);
         Serial.print(F("  ("));
         Serial.print((curStage < 0 ? -curStage : curStage) * DEG_PER_STAGE, 0);
         Serial.print(F("도 "));
         Serial.print(curStage < 0 ? F("왼쪽") : F("오른쪽"));
         Serial.println(F(")")); }
  {
    int32_t a = readAdc();
    Serial.print(F("조향각(ROS +우) : "));
    float d = adcToDegRos(a);
    if (d >= 0) Serial.print('+');
    Serial.print(d, 1); Serial.println(F(" 도"));
    Serial.print(F("  ADC 증가 = 좌회전 = ROS 각도 감소"));
    Serial.println();
  }
  Serial.print(F("이동중        : ")); Serial.println(moving ? F("YES") : F("NO"));
  Serial.print(F("마지막 결과   : ")); Serial.println(lastResult);
  Serial.print(F("방향 반전     : ")); Serial.println(dirInvert ? 1 : 0);
  Serial.print(F("미세이동 폭   : ")); Serial.println(nudgeStep);

  Serial.println(F("--- 프리셋 표 (단수:ADC, 1단=1도) ---"));
  {
    uint8_t col = 0;
    for (int8_t s = -STEER_STAGES; s <= STEER_STAGES; s++) {
      int32_t v = stageToAdc(s);
      if (s == 0) Serial.print(F("  C  :"));
      else {
        Serial.print(F("  "));
        Serial.print(s < 0 ? 'L' : 'R');
        int8_t a = s < 0 ? -s : s;
        if (a < 10) Serial.print(' ');
        Serial.print(a);
        Serial.print(':');
      }
      if (v < 100) Serial.print(' ');
      if (v < 10)  Serial.print(' ');
      Serial.print(v);
      if (v < (int32_t)ADC_HARD_LO || v > (int32_t)ADC_HARD_HI) Serial.print('!');
      else if (s == curStage) Serial.print('*');
      else Serial.print(' ');

      if (++col >= 5) { Serial.println(); col = 0; }
    }
    if (col != 0) Serial.println();
    Serial.println(F("  (* = 현재 단수,  ! = 하드리밋 밖)"));
  }

  Serial.println(F("--- 구동 ---"));
  Serial.print(F("PWM           : ")); Serial.println(drivePwm);
  Serial.print(F("방향          : ")); Serial.println(dirForward ? F("FWD") : F("REV"));
  Serial.print(F("주행중        : ")); Serial.println(running ? F("YES") : F("NO"));

  Serial.println(F("--- 엔코더 ---"));
  Serial.print(F("countA        : ")); Serial.println(a);
  Serial.print(F("countB        : ")); Serial.println(b);
  Serial.print(F("초당 카운트   : ")); Serial.println(rateA);
  if (a > 0) {
    Serial.print(F("추정 이동거리 : "));
    Serial.print((float)a / 199.8f, 3); Serial.println(F(" m  (c/m 199.8 기준)"));
  }
  Serial.println(F("======================================"));
  Serial.println();
}

void printStatus() {
  noInterrupts(); uint32_t a = countA; interrupts();
  uint32_t now = millis();
  uint32_t dt = now - lastRateMs;
  if (dt >= 1000) { rateA = (a - prevA) * 1000UL / dt; prevA = a; lastRateMs = now; }

  Serial.print(F("ST,adc="));    Serial.print(readAdc());
  Serial.print(F(",ctr="));      Serial.print(centerAdc);
  Serial.print(F(",tgt="));      Serial.print(target);
  Serial.print(F(",stg="));
  if (curStage == 0) Serial.print('C');
  else { Serial.print(curStage < 0 ? 'L' : 'R'); Serial.print(curStage < 0 ? -curStage : curStage); }
  Serial.print(F(",encA="));     Serial.print(a);
  Serial.print(F(",rate="));     Serial.print(rateA);
  Serial.print(F("/s,pwm="));    Serial.print(drivePwm);
  Serial.print(F(",dir="));      Serial.println(dirForward ? F("F") : F("R"));
}

// ---------- 명령 ----------
void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    // ★ 즉시 정지 — 명령 첫 글자일 때만 (G100 의 0 에 반응하지 않게)
    if (isStopChar(c)) {
      cmdLen = 0;
      abortTestRun();                 // 5초 시험 중이면 같이 중단
      driveStop(F("USER"));
      if (moving) stopMove("USER_STOP"); else steerOff();
      Serial.println(F("*** STOP ***"));
      continue;
    }

    // ★ 10초 주행 시험 — 명령 첫 글자일 때만 (G100 / L1 의 1 에 반응하지 않게)
    if (cmdLen == 0 && c == '1') {
      startTestRun(TEST_RUN_MS);          // 10초
      continue;
    }
    // ★ v10 — 5초 주행 시험
    if (cmdLen == 0 && c == '2') {
      startTestRun(TEST_RUN_MS_5S);       // 5초
      continue;
    }
    // ★ v10 — ADC 카운트 단위 미세 이동 (첫 글자일 때만)
    //   L/R 은 "도" 단위, 이건 "ADC 카운트" 단위다.
    //   중앙을 정밀하게 찾을 때 쓴다. 1카운트 = 약 0.069도.
    //   x / X / 0 은 v9 부터 "즉시 정지" 다. 미세이동에 쓰지 않는다.
    //   ★ 전부 기호다. v9 의 알파벳 명령(W/E/R/B/L/R/G/K/S...)과 절대 안 겹친다.
    if (cmdLen == 0 && (c==','||c=='.'||c=='<'||c=='>'||c=='['||c==']')) {
      int delta = 0;
      switch (c) {
        case ',': delta =  -1; break;    // 좌 1
        case '.': delta =  +1; break;    // 우 1
        case '<': delta =  -5; break;    // 좌 5
        case '>': delta =  +5; break;    // 우 5
        case '[': delta = -10; break;    // 좌 10
        case ']': delta = +10; break;    // 우 10
      }
      startMove((int32_t)readAdc() + delta, curStage);
      continue;
    }
    // ★ v10 — E-Stop 걸기 / 해제 (첫 글자일 때만)
    //   ★ E / R 은 v9 에서 전진100 / 전진150 이다. 기호를 쓴다.
    if (cmdLen == 0 && c == '!') {              // ! = E-Stop 걸기
      estopLatched = true;
      estopStopEverything();
      Serial.println(F("*** E-STOP (소프트) *** 래치됨.  ~ 로 해제"));
      continue;
    }
    if (cmdLen == 0 && c == '~') {              // ~ = 래치 해제
      if (digitalRead(ESTOP_PIN) == HIGH) {
        Serial.println(F("ERR,E-Stop 버튼이 아직 눌려 있다 (또는 단선)"));
      } else {
        estopLatched = false;
        Serial.println(F("E-Stop 래치 해제"));
      }
      continue;
    }
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) {
        char cmd = cmdBuf[0];
        int  val = atoi(cmdBuf + 1);

        // --- 조향 프리셋 ---
        if ((cmd == 'L' || cmd == 'l') && val >= 1 && val <= STEER_STAGES) {
          startMove(stageToAdc(-(int8_t)val), -(int8_t)val);
        }
        else if ((cmd == 'R' || cmd == 'r') && val >= 1 && val <= STEER_STAGES) {
          startMove(stageToAdc((int8_t)val), (int8_t)val);
        }
        //  ★ v12 — r 단독 '전진 150' 제거.
        //     구동은 w / b / e 5단위로 일원화했다. r 은 조향 우측 프리셋 전용.
        else if (cmd == 'C' || cmd == 'c') startMove(centerAdc, 0);
        //  ★ v12 — J : 기록값과 무관하게 실측 기준 중앙(355)으로.
        //     K 로 잘못 기록했을 때 되돌아오는 안전핀이다.
        else if (cmd == 'J' || cmd == 'j') {
          Serial.print(F("기준 중앙 ")); Serial.print(STEER_CENTER_DEFAULT);
          Serial.println(F(" 으로 (K 기록과 무관)"));
          startMove(STEER_CENTER_DEFAULT, 0);
        }
        else if (cmd == 'G' || cmd == 'g') startMove(val, 0);
        else if (cmd == 'a' || cmd == 'A') startMove((int32_t)readAdc() - nudgeStep, curStage);
        else if (cmd == 'd' || cmd == 'D') startMove((int32_t)readAdc() + nudgeStep, curStage);
        else if (cmd == 'N') {
          if (val >= 1 && val <= 100) { nudgeStep = (uint8_t)val;
            Serial.print(F("NUDGE,")); Serial.println(nudgeStep); }
          else Serial.println(F("ERR,1~100"));
        }
        else if (cmd == 'K' || cmd == 'k') {
          centerAdc = readAdc();
          curStage = 0;
          target = centerAdc;
          Serial.println();
          Serial.print(F("★ CENTER_SET,")); Serial.println(centerAdc);
          Serial.println(F("   이 값을 v38 STEER_CENTER_ADC 에 넣어라"));
          Serial.print(F("   새 프리셋 범위 : "));
          Serial.print(stageToAdc(-STEER_STAGES)); Serial.print(F(" ~ "));
          Serial.println(stageToAdc(STEER_STAGES));
          Serial.println();
        }
        else if (cmd == 'I' || cmd == 'i') {
          dirInvert = !dirInvert; dirLearned = true;
          Serial.print(F("INVERT,")); Serial.println(dirInvert ? 1 : 0);
        }
        // --- 주행 ---
        //  ★ v11 — 5 PWM 단위 증감. 누를 때마다 즉시 인가한다.
        else if (cmd == 'w' || cmd == 'W') driveStep(true,  +PWM_STEP);
        else if (cmd == 'b' || cmd == 'B') driveStep(false, +PWM_STEP);
        else if (cmd == 'e' || cmd == 'E') driveStep(dirForward, -PWM_STEP);
        // --- 엔코더 / 기타 ---
        else if (cmd == 'o' || cmd == 'O') resetCounts();
        else if (cmd == 'M' || cmd == 'm') printCpm(atof(cmdBuf + 1));
        else if (cmd == 'S' || cmd == 's') printFullLog();
        else if (cmd == 'h' || cmd == 'H' || cmd == '?') printHelp();
        else Serial.println(F("ERR,모르는 명령. h"));
      }
      cmdLen = 0;
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
}

// ---------- setup ----------
void setup() {
  Serial.begin(BAUD);

  pinMode(PWM_DRIVE_FRONT, OUTPUT); pinMode(DIR_DRIVE_FRONT, OUTPUT);
  pinMode(PWM_DRIVE_REAR,  OUTPUT); pinMode(DIR_DRIVE_REAR,  OUTPUT);
  applyDrivePwm(0);
  dirForward = true; applyDriveDir();

  pinMode(PWM_STEER, OUTPUT); pinMode(DIR_STEER, OUTPUT);
  steerOff();

  pinMode(PIN_POT, INPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);           // ★ v10
  lastEstopReading = (digitalRead(ESTOP_PIN) == HIGH);
  estopActive      = lastEstopReading;
  estopLatched     = lastEstopReading;
  pinMode(ENC_B, INPUT_PULLUP);

  lastAUs = lastBUs = micros();
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, RISING);

  lastRateMs = millis();
  tPrint = millis();

  delay(300);
  Serial.println();
  Serial.println(F("FIELD,v9,0904"));
  Serial.println(F("STOP_KEY,0 (x 도 가능)  /  TEST_RUN_KEY,1 (10초 주행)"));
  Serial.println(F("RAMP,DISABLED (명령 즉시 앞뒤 동시 인가)"));
  Serial.print  (F("STEER,center=")); Serial.print(centerAdc);
  Serial.print  (F(",deg_per_count=")); Serial.print(COUNTS_PER_DEG, 1);
  Serial.print  (F(",stages=±"));     Serial.print(STEER_STAGES);
  Serial.print  (F("도,range="));      Serial.print(stageToAdc(STEER_STAGES));
  Serial.print  (F("~"));             Serial.println(stageToAdc(-STEER_STAGES));
  Serial.print  (F("HARD_LIMIT,"));   Serial.print(ADC_HARD_LO);
  Serial.print  (F("~"));             Serial.println(ADC_HARD_HI);
  Serial.println(F("ENC,하드웨어 RISING 인터럽트 (v37 동일)"));

  // --- 리밋 밖 프리셋 경고 ---
  {
    uint8_t bad = 0;
    for (int8_t s = -STEER_STAGES; s <= STEER_STAGES; s++) {
      int32_t v = stageToAdc(s);
      if (v < (int32_t)ADC_HARD_LO || v > (int32_t)ADC_HARD_HI) {
        if (bad == 0) Serial.print(F("⚠ 리밋 밖 프리셋 : "));
        else Serial.print(F(", "));
        Serial.print(s < 0 ? 'L' : 'R'); Serial.print(s < 0 ? -s : s);
        Serial.print(F("(")); Serial.print(v); Serial.print(F(")"));
        bad++;
      }
    }
    if (bad > 0) {
      Serial.println();
      Serial.println(F("   이 단수는 명령해도 거부된다."));
      Serial.println(F("   좌우 여유가 다르면 중앙이 치우친 것이다."));
      Serial.print(F("   왼쪽 여유 "));  Serial.print((int32_t)centerAdc - ADC_HARD_LO);
      Serial.print(F(" / 오른쪽 여유 ")); Serial.println((int32_t)ADC_HARD_HI - centerAdc);
    } else {
      Serial.println(F("프리셋 전부 리밋 안쪽 OK"));
    }
  }
  Serial.println();
  printHelp();
  Serial.println();
  Serial.print(F("현재 ADC : ")); Serial.println(readAdc());
  Serial.println(F("준비 완료. h 를 치면 명령 목록이 나온다."));
  Serial.println();
}

// ---------- loop ----------
void loop() {
  updateEstop();      // ★ v10 — 무엇보다 먼저
  handleSerial();

  updateTestRun();

  if (moving) stepMove();

  if (running && millis() - cmdTime > AUTO_STOP_MS && drivePwm != 0) {
    driveStop(F("AUTO_30초"));
  }

  if (millis() - tPrint >= PRINT_MS) {
    tPrint = millis();
    if (!moving) printStatus();
  }
}
