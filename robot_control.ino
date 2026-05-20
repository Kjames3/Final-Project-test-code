// ================================================================
// JUMPING WHEEL-LEGGED ROBOT — Arduino Uno R3
// UCR MEDDL Lab
// ================================================================
// PIN MAP:
//  D2  - Left encoder A  (INT0 hardware interrupt)
//  D3~ - AT8236 AIN2     (left motor direction 2)
//  D4  - AT8236 AIN1     (left motor direction 1)
//  D5~ - AT8236 PWMA     (left motor speed)
//  D6~ - AT8236 PWMB     (right motor speed)
//  D7  - AT8236 BIN1     (right motor direction 1)
//  D8  - Left encoder B
//  D9~ - AT8236 BIN2     (right motor direction 2)
//  D10 - URT-2 TX (SoftwareSerial)
//  D11 - URT-2 RX (SoftwareSerial)
//  D12 - Right encoder A
//  D13 - Right encoder B
//  A4  - MPU6050 SDA
//  A5  - MPU6050 SCL
// ================================================================
// REQUIRED LIBRARIES (install via Arduino IDE Library Manager):
//   1. Wire.h         — built-in (I2C for MPU6050)
//   2. SoftwareSerial — built-in
//   3. SCServo        — search "SCServo" by FEETECH in Library Manager
// ================================================================
// BEFORE FIRST RUN:
//   - Set FEETECH servo baud rate to 115200 using URT-2 + PC software
//   - Set servo IDs: left hip = 1, right hip = 2
//   - Calibrate BALANCE_OFFSET so robot stands straight
// ================================================================

#include <Wire.h>
#include <SoftwareSerial.h>
#include <SCServo.h>

// ── TUNING PARAMETERS ─────────────────────────────────────────
// Adjust these to tune the balance and response

// LQR / Balance gains
#define KP_ANGLE       45.0    // Proportional gain on tilt angle
#define KD_ANGLE       1.8     // Derivative gain on tilt rate
#define KP_SPEED       18.0    // Proportional gain on speed error
#define KI_SPEED       0.8     // Integral gain on speed error

// Tilt offset (degrees) — tune so robot stands upright without moving
// Positive = robot leans forward, Negative = leans back
#define BALANCE_OFFSET  2.5

// Max robot speed from controller (motor PWM units 0-255)
#define MAX_SPEED      180
#define MAX_TURN        80

// Fall detection — stop motors if tilted too far
#define FALL_ANGLE      35.0   // degrees

// Encoder pulses per revolution (full quadrature)
// JGB37-520: typically 11 pulses/rev on motor shaft x gear ratio
// Measure by spinning wheel one full rotation and counting pulses
#define ENC_PPR         660    // adjust after measuring

// Wheel circumference (metres)
#define WHEEL_CIRC      0.204  // π × 0.065m diameter

// Complementary filter coefficient (0.95-0.99)
#define CF_ALPHA        0.98

// MPU6050 I2C address
#define MPU_ADDR        0x68

// Loop timing
#define LOOP_HZ         200    // balance loop frequency
#define LOOP_US         (1000000 / LOOP_HZ)

// Jump parameters (servo positions 0-4095 = 0-360°)
// Adjust these after physical testing
#define HIP_NEUTRAL_L   2048   // Standing position left hip
#define HIP_NEUTRAL_R   2048   // Standing position right hip  
#define HIP_CROUCH_L    1200   // Crouch position left (energy storage)
#define HIP_CROUCH_R    2896   // Crouch position right (mirrored)
#define HIP_JUMP_L      2800   // Jump release left
#define HIP_JUMP_R      1296   // Jump release right (mirrored)
#define HIP_SPEED       3000   // Servo speed for jump (max = 4095)
#define HIP_ACCEL       200    // Servo acceleration
#define HIP_BAUD        115200 // Must match what you set via URT-2 PC tool

// ── PIN DEFINITIONS ───────────────────────────────────────────
#define PIN_ENC_L_A     2      // INT0
#define PIN_ENC_L_B     8
#define PIN_ENC_R_A     12
#define PIN_ENC_R_B     13
#define PIN_AIN2        3
#define PIN_AIN1        4
#define PIN_PWMA        5
#define PIN_PWMB        6
#define PIN_BIN1        7
#define PIN_BIN2        9
#define PIN_SERVO_TX    10
#define PIN_SERVO_RX    11

// ── GLOBALS ───────────────────────────────────────────────────
// Servo
SoftwareSerial servoSerial(PIN_SERVO_RX, PIN_SERVO_TX);
SMS_STS hip;

// Encoder counts (volatile — modified in ISR)
volatile long encL = 0;
volatile long encR = 0;

// MPU6050 raw values
int16_t ax, ay, az, gx, gy, gz;

// Balance state
float tiltAngle    = 0.0;   // degrees, 0 = vertical
float tiltRate     = 0.0;   // degrees/sec
float speedL       = 0.0;   // m/s left wheel
float speedR       = 0.0;   // m/s right wheel
float speedAvg     = 0.0;   // average forward speed
float speedInteg   = 0.0;   // speed error integral
long  prevEncL     = 0;
long  prevEncR     = 0;

// Controller commands (received from RPi over serial)
int   cmdSpeed     = 0;     // -255 to 255 (forward/backward)
int   cmdTurn      = 0;     // -255 to 255 (left/right)
bool  cmdJump      = false;

// State flags
bool  fallen       = false;
bool  jumping      = false;
unsigned long jumpStartMs = 0;

// Timing
unsigned long prevLoopUs   = 0;
unsigned long prevSpeedMs  = 0;
unsigned long prevSerialMs = 0;

// ── ENCODER ISR ───────────────────────────────────────────────
void IRAM_ATTR encL_ISR() {
  // Count left encoder pulses (using A channel interrupt)
  if (digitalRead(PIN_ENC_L_B) == LOW) encL++;
  else                                  encL--;
}

// ── MPU6050 FUNCTIONS ─────────────────────────────────────────
void mpuInit() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz fast mode

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s range
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g range
  Wire.endTransmission();

  // Digital low-pass filter — 44Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  // CONFIG
  Wire.write(0x03);
  Wire.endTransmission();
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ── ANGLE CALCULATION (Complementary Filter) ──────────────────
// Returns angle in degrees. 0 = upright, + = leaning forward
void updateAngle(float dt) {
  // Accelerometer angle (X-axis tilt)
  float accelAngle = atan2((float)ay, (float)az) * 57.2958;

  // Gyro rate (X-axis rotation rate in deg/s)
  tiltRate = (float)gx / 131.0;  // 131 LSB/deg/s for ±250°/s

  // Complementary filter
  tiltAngle = CF_ALPHA * (tiltAngle + tiltRate * dt) +
              (1.0 - CF_ALPHA) * accelAngle;
}

// ── MOTOR CONTROL ─────────────────────────────────────────────
void setMotors(int leftPWM, int rightPWM) {
  // Left motor (Channel A)
  if (leftPWM > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, constrain(leftPWM, 0, 255));
  } else if (leftPWM < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, constrain(-leftPWM, 0, 255));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
  }

  // Right motor (Channel B)
  if (rightPWM > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, constrain(rightPWM, 0, 255));
  } else if (rightPWM < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, constrain(-rightPWM, 0, 255));
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, 0);
  }
}

void stopMotors() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// ── SPEED MEASUREMENT ─────────────────────────────────────────
void updateSpeed(float dt) {
  long dL = encL - prevEncL;
  long dR = encR - prevEncR;
  prevEncL = encL;
  prevEncR = encR;

  // Convert pulses to m/s
  speedL = ((float)dL / ENC_PPR) * WHEEL_CIRC / dt;
  speedR = ((float)dR / ENC_PPR) * WHEEL_CIRC / dt;
  speedAvg = (speedL + speedR) / 2.0;
}

// ── LQR BALANCE CONTROLLER ────────────────────────────────────
// Returns base PWM output (applied to both motors for balance)
int balanceControl(float dt) {
  // Target speed from controller (map -255..255 to m/s target)
  float targetSpeed = (float)cmdSpeed / 255.0 * 1.5;  // max 1.5 m/s

  // Speed error
  float speedError = speedAvg - targetSpeed;
  speedInteg += speedError * dt;
  speedInteg = constrain(speedInteg, -50.0, 50.0);  // anti-windup

  // Error angle (add balance offset so robot stands upright)
  float angleError = (tiltAngle - BALANCE_OFFSET)
                   + KP_SPEED  * speedError
                   + KI_SPEED  * speedInteg;

  // PD on angle
  int output = (int)(KP_ANGLE * angleError + KD_ANGLE * tiltRate);
  return constrain(output, -255, 255);
}

// ── JUMP CONTROL ──────────────────────────────────────────────
void startJump() {
  if (jumping) return;
  jumping = true;
  jumpStartMs = millis();

  // Phase 1: Crouch (store energy)
  hip.WritePosEx(1, HIP_CROUCH_L, 1500, 50);
  hip.WritePosEx(2, HIP_CROUCH_R, 1500, 50);
}

void updateJump() {
  if (!jumping) return;
  unsigned long elapsed = millis() - jumpStartMs;

  if (elapsed < 300) {
    // Phase 1: Crouching — hold position
    // Nothing to do, servos moving to crouch
  } else if (elapsed < 500) {
    // Phase 2: Release — rapid hip extension
    hip.WritePosEx(1, HIP_JUMP_L, HIP_SPEED, HIP_ACCEL);
    hip.WritePosEx(2, HIP_JUMP_R, HIP_SPEED, HIP_ACCEL);
  } else if (elapsed < 900) {
    // Phase 3: Flight — contract to improve height
    hip.WritePosEx(1, HIP_CROUCH_L, HIP_SPEED, HIP_ACCEL);
    hip.WritePosEx(2, HIP_CROUCH_R, HIP_SPEED, HIP_ACCEL);
  } else if (elapsed < 1400) {
    // Phase 4: Landing prep — extend back to neutral
    hip.WritePosEx(1, HIP_NEUTRAL_L, 1000, 50);
    hip.WritePosEx(2, HIP_NEUTRAL_R, 1000, 50);
  } else {
    // Phase 5: Done
    jumping = false;
  }
}

void setHipsNeutral() {
  hip.WritePosEx(1, HIP_NEUTRAL_L, 800, 30);
  hip.WritePosEx(2, HIP_NEUTRAL_R, 800, 30);
}

// ── SERIAL COMMAND PARSER ─────────────────────────────────────
// Protocol from RPi: "CMD:<speed>:<turn>:<jump>\n"
// Example: "CMD:120:-50:0\n"
void parseSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("CMD:")) return;

  // Parse CMD:<speed>:<turn>:<jump>
  int idx1 = line.indexOf(':', 4);
  int idx2 = line.indexOf(':', idx1 + 1);

  if (idx1 < 0 || idx2 < 0) return;

  int spd  = line.substring(4,    idx1).toInt();
  int turn = line.substring(idx1+1, idx2).toInt();
  int jmp  = line.substring(idx2+1).toInt();

  cmdSpeed = constrain(spd,  -255, 255);
  cmdTurn  = constrain(turn, -255, 255);

  if (jmp == 1 && !cmdJump) {
    cmdJump = true;
    startJump();
  } else if (jmp == 0) {
    cmdJump = false;
  }
}

// ── SEND TELEMETRY TO RPi ─────────────────────────────────────
void sendTelemetry() {
  Serial.print("TEL:");
  Serial.print(tiltAngle, 1);
  Serial.print(":");
  Serial.print(speedAvg * 100, 0);  // cm/s
  Serial.print(":");
  Serial.print(fallen ? "1" : "0");
  Serial.print(":");
  Serial.println(jumping ? "1" : "0");
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);  // USB serial to RPi

  // Motor pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  stopMotors();

  // Encoder pins
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Left encoder interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A),
                  encL_ISR, CHANGE);

  // MPU6050
  mpuInit();
  delay(100);

  // Warm up angle filter (let it settle for 1 second)
  // Read MPU many times so complementary filter converges
  for (int i = 0; i < 200; i++) {
    mpuRead();
    float accelAngle = atan2((float)ay, (float)az) * 57.2958;
    tiltAngle = 0.8 * tiltAngle + 0.2 * accelAngle;
    delay(5);
  }

  // FEETECH servos
  servoSerial.begin(HIP_BAUD);
  hip.pSerial = &servoSerial;
  delay(200);
  setHipsNeutral();

  Serial.println("READY");

  prevLoopUs = micros();
  prevSpeedMs = millis();
}

// ── MAIN LOOP ─────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if ((now - prevLoopUs) < LOOP_US) return;  // Rate limit

  float dt = (now - prevLoopUs) / 1000000.0;
  prevLoopUs = now;
  dt = constrain(dt, 0.001, 0.05);  // Safety clamp

  // 1. Read IMU and update angle
  mpuRead();
  updateAngle(dt);

  // 2. Update wheel speed
  updateSpeed(dt);

  // 3. Parse any incoming serial commands
  parseSerial();

  // 4. Check for fall
  if (abs(tiltAngle - BALANCE_OFFSET) > FALL_ANGLE) {
    fallen = true;
    stopMotors();
    speedInteg = 0.0;
  } else {
    fallen = false;
  }

  // 5. Update jump state machine
  updateJump();

  // 6. Compute and apply motor control (only if not fallen)
  if (!fallen) {
    int basePWM = balanceControl(dt);

    // Turn: differential — left faster = turn right
    int turnPWM = map(cmdTurn, -255, 255, -MAX_TURN, MAX_TURN);

    int leftPWM  = basePWM - turnPWM;
    int rightPWM = basePWM + turnPWM;

    leftPWM  = constrain(leftPWM,  -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    setMotors(leftPWM, rightPWM);
  }

  // 7. Send telemetry every 50ms
  if ((millis() - prevSerialMs) >= 50) {
    sendTelemetry();
    prevSerialMs = millis();
  }
}
