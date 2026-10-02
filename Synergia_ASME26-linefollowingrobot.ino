// ============================================================
// ESP32 5-Sensor Analog IR Line Follower + TB6612FNG
// Auto-calibrating sensors, BLACK line
// Lost line: ~50 deg search for 3 s, then two-turn scan:
//   TURN1 (toward last side) -> TURN2 (other side, only if TURN1 failed)
// Nothing found after both turns -> motors HARD STOP
// ============================================================

// ---------------- PINS ----------------
const int   IR_PINS[5] = {27, 26, 25, 33, 32};   // Far Left -> Far Right
const float IR_POS[5]  = {-2, -1, 0, 1, 2};

#define PWMA 23
#define AIN2 22
#define AIN1 21
#define STBY 19
#define BIN1 18
#define BIN2 5
#define PWMB 17

// ---------------- MODES ----------------
#define DEBUG_ONLY 0     // 1 = motors OFF, only print sensor data
#define CAL_RUN    0     // 1 = speed test run (see steps below), then stops

// Sensor polarity: 1 = black reads HIGH, 0 = black reads LOW, -1 = auto-detect
// Start with 1. If sensors read ~1000 on white floor, change to 0.
#define FORCE_POLARITY 1

// ---------------- SETTINGS ----------------
const int STEER_SIGN = 1;     // -1 if robot turns AWAY from the line
const bool LEFT_FWD_IN1_HIGH  = false;
const bool RIGHT_FWD_IN1_HIGH = true;

// NOTE: 84/255 is only ~33% power. For ~70% use about 180,
// then retune KP / KD / MAX_CORRECTION.
const int BASE_SPEED = 84;
const int MAX_SPEED  = 255;

const float KP = 20.0;
const float KD = 10.0;
const int   MAX_CORRECTION = 63;

// A line under a 5-sensor array covers at most this many sensors.
// If more sensors than this read "black", it's a calibration/lift-off problem.
const int MAX_ACTIVE_SENSORS = 3;

// ---------------- TURN-ANGLE CALIBRATION ----------------
const int TURN_SPEED = 70;                 // PWM used for search spin

const float TRACK_MM         = 170.0;      // wheel center to center = 17 cm
const float SEARCH_ANGLE_DEG = 50.0;       // quick search angle (first 3 s)
const float SCAN_ANGLE_DEG   = 90.0;       // two-turn scan angle each side (after 3 s)
const float ANGLE_TRIM       = 1.0;        // >1 = turn more, <1 = turn less

// Speed test: CAL_RUN=1 drives straight at TURN_SPEED for CAL_RUN_MS.
// Measure the distance it covered (cm) and put it below.
const int   CAL_RUN_MS   = 2000;
const float MEASURED_CM  = 30.0;           // <-- REPLACE with your measured value

// Derived (don't edit)
const float WHEEL_SPEED_MM_S = (MEASURED_CM * 10.0) / (CAL_RUN_MS / 1000.0);
const int   SEARCH_MS = (int)((SEARCH_ANGLE_DEG * PI / 180.0) * TRACK_MM
                              / (2.0 * WHEEL_SPEED_MM_S) * 1000.0 * ANGLE_TRIM);
const int   SCAN_MS   = (int)((SCAN_ANGLE_DEG * PI / 180.0) * TRACK_MM
                              / (2.0 * WHEEL_SPEED_MM_S) * 1000.0 * ANGLE_TRIM);

const unsigned long SCAN_AFTER_MS = 3000;     // no line for 3 s -> two-turn scan

const int CAL_TIME_MS = 4000;                 // sensor calibration time
const bool DEFAULT_BLACK_IS_HIGH = true;

// ---------------- STATE ----------------
int  mn[5], mx[5];
long sm[5];
bool blackHigh = DEFAULT_BLACK_IS_HIGH;

float lastPos = 0;
int   lastDir = 0;            // -1 line was left, 1 right, 0 unknown
unsigned long lostSince = 0;
bool wasLost = false;
unsigned long lastPrint = 0;
unsigned long lastLoop = 0;

float headMs = 0;             // estimated turn since line lost (+ = toward lastDir)
int   scanStage = 0;          // 0 none, 1 = first turn, 2 = second turn, 3 = done

// ---------------- MOTORS ----------------
void setMotor(int in1, int in2, int pwmPin, int speed, bool fwdIn1High)
{
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (speed > 0)
  {
    digitalWrite(in1, fwdIn1High);
    digitalWrite(in2, !fwdIn1High);
    analogWrite(pwmPin, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(in1, !fwdIn1High);
    digitalWrite(in2, fwdIn1High);
    analogWrite(pwmPin, -speed);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void drive(int left, int right)
{
  setMotor(AIN1, AIN2, PWMA, left,  LEFT_FWD_IN1_HIGH);
  setMotor(BIN1, BIN2, PWMB, right, RIGHT_FWD_IN1_HIGH);
}

// Hard stop: TB6612 short-brake (both inputs HIGH) so the robot doesn't coast
void stopMotors()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
}

// ---------------- SENSOR CALIBRATION ----------------
void calibrate()
{
  for (int i = 0; i < 5; i++) { mn[i] = 4095; mx[i] = 0; sm[i] = 0; }

  Serial.println("CALIBRATING: slide robot across the black line now...");

  unsigned long t0 = millis();
  long n = 0;

  while (millis() - t0 < CAL_TIME_MS)
  {
    for (int i = 0; i < 5; i++)
    {
      int v = analogRead(IR_PINS[i]);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
      sm[i] += v;
    }
    n++;
    delay(2);
  }

  int vote = 0;
  for (int i = 0; i < 5; i++)
  {
    if (mx[i] - mn[i] > 300)
    {
      float mean = sm[i] / (float)n;
      if (mean - mn[i] < mx[i] - mean) vote++;
      else vote--;
    }
  }

#if FORCE_POLARITY == -1
  if (vote > 0)      blackHigh = true;
  else if (vote < 0) blackHigh = false;
  else               blackHigh = DEFAULT_BLACK_IS_HIGH;
#else
  blackHigh = (FORCE_POLARITY == 1);
#endif

  Serial.print("Black is ");
  Serial.println(blackHigh ? "HIGH" : "LOW");

  for (int i = 0; i < 5; i++)
    Serial.printf("S%d min:%d max:%d\n", i + 1, mn[i], mx[i]);
}

// 0..1000, 1000 = black line
int readNorm(int i)
{
  int raw   = analogRead(IR_PINS[i]);
  int range = max(mx[i] - mn[i], 300);
  long v = (long)(raw - mn[i]) * 1000 / range;
  v = constrain(v, 0, 1000);
  if (!blackHigh) v = 1000 - v;
  return (int)v;
}

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) pinMode(IR_PINS[i], INPUT);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  drive(0, 0);

  analogReadResolution(12);

#if CAL_RUN
  Serial.println("SPEED TEST: put robot on floor with a start mark. Driving in 3 s...");
  delay(3000);
  drive(TURN_SPEED, TURN_SPEED);
  delay(CAL_RUN_MS);
  stopMotors();
  Serial.println("Done. Measure distance travelled (cm) -> MEASURED_CM, set CAL_RUN 0.");
  while (true) delay(1000);
#endif

  delay(2000);

  calibrate();

  Serial.printf("Wheel speed %.0f mm/s -> search %d ms (%.0f deg), scan %d ms (%.0f deg)\n",
                WHEEL_SPEED_MM_S, SEARCH_MS, SEARCH_ANGLE_DEG, SCAN_MS, SCAN_ANGLE_DEG);
  Serial.println("Place robot on the line. Starting in 3 s...");
  delay(3000);

  lastDir = 0;
  lastPos = 0;
  wasLost = false;
  lastLoop = millis();
}

// ---------------- LOOP ----------------
void loop()
{
  unsigned long now = millis();
  unsigned long dt  = now - lastLoop;
  lastLoop = now;

  int  v[5];
  long total = 0;
  float weighted = 0;
  int  maxv = 0;
  int  active = 0;

  for (int i = 0; i < 5; i++)
  {
    v[i] = readNorm(i);
    if (v[i] > maxv) maxv = v[i];
    if (v[i] > 400) active++;

    int w = (v[i] > 200) ? v[i] : 0;
    total    += w;
    weighted += (float)w * IR_POS[i];
  }

  // Too many sensors "black" = bad calibration / lifted robot, not a real line
  bool onLine = (maxv > 400) && (total > 0) && (active <= MAX_ACTIVE_SENSORS);
  float pos = 0;
  int leftSpeed = 0, rightSpeed = 0;
  const char* state = "LINE";

#if DEBUG_ONLY
  if (onLine) pos = STEER_SIGN * weighted / total;
#else
  if (onLine)
  {
    // Line sensed (also mid-turn): reset scan and carry on following
    wasLost   = false;
    headMs    = 0;
    scanStage = 0;

    pos = STEER_SIGN * weighted / total;      // -2 left ... +2 right
    float correction = KP * pos + KD * (pos - lastPos);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    lastPos = pos;

    if (pos < -0.1)     lastDir = -1;
    else if (pos > 0.1) lastDir = 1;

    leftSpeed  = constrain(BASE_SPEED + (int)correction, 0, MAX_SPEED);
    rightSpeed = constrain(BASE_SPEED - (int)correction, 0, MAX_SPEED);
    drive(leftSpeed, rightSpeed);
  }
  else
  {
    if (!wasLost)
    {
      wasLost   = true;
      lostSince = now;
      headMs    = 0;
      scanStage = 0;
    }

    unsigned long t = now - lostSince;
    int dir = lastDir;
    int sp  = 0;            // spin: +1 toward last side, -1 away, 0 none

    if (dir == 0)
    {
      state = "WAIT";                           // never saw line -> stay still
    }
    else if (t < (unsigned long)SEARCH_MS)
    {
      state = "SEARCH1";                        // quick turn toward last side
      sp = 1;
    }
    else if (t < SCAN_AFTER_MS)
    {
      state = "SWEEP";                          // +-50 deg swing (first 3 s)
      int phase = (t - SEARCH_MS) / (2 * SEARCH_MS);
      sp = (phase % 2 == 0) ? -1 : 1;
    }
    else
    {
      // 3 s without line: two-turn scan. Stops turning the moment line is sensed
      // (onLine branch above), so TURN2 only happens if TURN1 found nothing.
      if (scanStage == 0) scanStage = 1;
      if (scanStage == 1 && headMs >=  SCAN_MS) scanStage = 2;
      if (scanStage == 2 && headMs <= -SCAN_MS) scanStage = 3;

      if (scanStage == 1)      { state = "TURN1"; sp =  1; }   // toward last side
      else if (scanStage == 2) { state = "TURN2"; sp = -1; }   // other side
      else                     { state = "STOP";  sp =  0; }   // nothing found
    }

    if (sp == 0)
    {
      stopMotors();                             // WAIT / STOP -> motors hard stop
    }
    else
    {
      leftSpeed  =  dir * sp * TURN_SPEED;
      rightSpeed = -dir * sp * TURN_SPEED;
      drive(leftSpeed, rightSpeed);
    }

    headMs += sp * (float)dt;                   // track how far we've turned
  }
#endif

  if (millis() - lastPrint > 150)
  {
    lastPrint = millis();
    Serial.printf("%4d %4d %4d %4d %4d | act:%d | pos:%5.2f | L:%4d R:%4d | %s\n",
                  v[0], v[1], v[2], v[3], v[4], active, pos,
                  leftSpeed, rightSpeed, state);
  }

  delay(5);
}