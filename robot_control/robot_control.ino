// ================================================================
// JUMPING WHEEL-LEGGED ROBOT — Arduino Uno R3
// UCR MEDDL Lab
// ================================================================
// PIN MAP:
//  D2  - Right encoder B (direction)
//  D4  - Left encoder B (direction)
//  D5~ - AT8236 BIN1  (right motor PWM, Timer 0 OC0B)
//  D6~ - AT8236 BIN2  (right motor PWM, Timer 0 OC0A)
//  D7  - Right encoder A (interrupt via PCINT2)
//  D8  - Left encoder A  (interrupt via PCINT0)
//  D9~ - AT8236 AIN1  (left motor PWM, Timer 1 OC1A)
//  D10~- AT8236 AIN2  (left motor PWM, Timer 1 OC1B)
//  D11~- BLS3355 Right hip servo PWM (bit-banged via Timer 2 ISR)
//  D12 - BLS3355 Left hip servo PWM  (bit-banged via Timer 2 ISR)
//  A4  - MPU6050 SDA (I2C)
//  A5  - MPU6050 SCL (I2C)
// ================================================================
// SERVO NOTE: Servo.h cannot be used — it steals Timer 1, which
//   is needed for analogWrite on D9/D10 (motor control).
//   Instead, Timer 2 CTC ISR generates 50 Hz servo pulses on D12/D11.
//   ISR tick: 20 μs  (prescaler=8, OCR2A=39)
//   Pulse range: 500–2500 μs (25–125 ticks), frame: 1000 ticks = 20 ms
// ================================================================
// REQUIRED LIBRARIES:
//   1. Wire.h — built-in (I2C for MPU6050)
// ================================================================
// BEFORE FIRST RUN:
//   - Calibrate BALANCE_OFFSET so robot stands straight
//   - Set SERVO_BOOT_LEFT_US / SERVO_BOOT_RIGHT_US from the calibrated
//     stance (must match bls.left/right default_us in config/robot.yaml)
// ================================================================

#include <Wire.h>

// ── TUNING PARAMETERS ─────────────────────────────────────────
// LQR state-feedback gains from scripts/compute_lqr_gain.py.
// State x = [posErr (m), velErr (m/s), pitch (rad), pitchRate (rad/s)]
// Output: tau (N·m) = Kx·posErr + Kv·velErr + Kp·pitch + Kd·pitchRate
// PWM = Ks · tau  (Ks in PWM per N·m — the main hardware-tuning knob)
// Signs: Kp,Kd > 0 (lean forward → drive forward); Kx,Kv < 0 (resist drift)
float Kx =  -63.2456;   // position-error gain  (N·m/m)
float Kv =  -71.8334;   // velocity-error gain  (N·m per m/s)
float Kp =  345.3348;   // pitch gain           (N·m/rad)
float Kd =   82.7683;   // pitch-rate gain      (N·m per rad/s)
float Ks =    7.8;      // torque→PWM scalar    (PWM per N·m) — TUNE ON HARDWARE

// Tilt offset (degrees) — tune so robot stands upright without moving
// Positive = robot leans forward, Negative = leans back
float balanceOffset = 1.4;

// Max robot speed from controller (motor PWM units 0-255)
#define MAX_SPEED      180
#define MAX_TURN       130   // raised from 80 — gives enough differential to overcome motor stiction

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

// ── PIN DEFINITIONS ───────────────────────────────────────────────
#define PIN_ENC_L_A     8      // PCINT0 (Port B)
#define PIN_ENC_L_B     4
#define PIN_ENC_R_A     7      // PCINT2 (Port D)
#define PIN_ENC_R_B     2

#define PIN_AIN1        9      // Left Motor PWM Direction 1 (OC1A / Timer 1)
#define PIN_AIN2        10     // Left Motor PWM Direction 2 (OC1B / Timer 1)
#define PIN_BIN1        5      // Right Motor PWM Direction 1 (OC0B / Timer 0)
#define PIN_BIN2        6      // Right Motor PWM Direction 2 (OC0A / Timer 0)

#define SERVO_PIN_LEFT  12     // BLS3355 left hip  (bit-banged in Timer 2 ISR)
#define SERVO_PIN_RIGHT 11     // BLS3355 right hip (bit-banged in Timer 2 ISR)

// ── SERVO PWM (Timer 2 ISR, 20 μs tick) ───────────────────────────
// Tick period: (OCR2A+1) * prescaler / F_CPU = 40 * 8 / 16e6 = 20 μs
// Servo frame: 1000 ticks = 20 ms (50 Hz)
// Pulse range: 25 ticks (500 μs) to 125 ticks (2500 μs)
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_NEUTRAL_US  1500
#define SERVO_FRAME_TICKS 1000   // 20 ms

// Boot/default stance — the pose the hips drive to at power-on and on ESTOP.
// Keep in sync with bls.left/right default_us in config/robot.yaml.
// Observed on hardware: the mirrored pulse pair (1540/1460) moved the legs
// in OPPOSITE directions (left up, right down), which means the same pulse
// offset moves both legs the SAME way. No mirroring about neutral — both
// hips get the same pulse.
// 1540 = both legs +5.4° UP (nearest 20 μs tick to the 5° target).
#define SERVO_BOOT_LEFT_US   1540   // left leg  +5.4° UP
#define SERVO_BOOT_RIGHT_US  1540   // right leg +5.4° UP (same pulse, no mirroring)

// ── TEMPORARY HIP RAISE/LOWER TEST ────────────────────────────
// Set SERVO_TEST_MODE to 1 to oscillate both hips up/down about the
// standing stance (verifies the servos lift and lower the body under
// load before enabling LQR). Set to 0 to restore normal operation
// (robot just holds the standing stance). Kept the ±20° sweep settings
// below so it can be re-enabled instantly.
#define SERVO_TEST_MODE      0
#define SERVO_TEST_CENTER_US SERVO_BOOT_LEFT_US  // oscillate about standing stance (1540)
#define SERVO_TEST_AMP_US    148   // ≈20° swing  (~7.4 µs/deg on a 270° servo)
#define SERVO_TEST_PERIOD_MS 1000  // one full up→down→up cycle (ms) — 2× faster

// Pulse widths commanded by RPi (μs). Written from main loop, read by ISR.
// uint16_t writes are NOT atomic on AVR — always update with cli/sei guard.
volatile uint16_t g_servoLeftUs  = SERVO_BOOT_LEFT_US;
volatile uint16_t g_servoRightUs = SERVO_BOOT_RIGHT_US;

ISR(TIMER2_COMPA_vect) {
  static uint16_t tick     = 0;
  static uint16_t leftEnd  = (SERVO_BOOT_LEFT_US + 10) / 20;
  static uint16_t rightEnd = (SERVO_BOOT_LEFT_US + 10) / 20 + (SERVO_BOOT_RIGHT_US + 10) / 20;

  if (tick == 0) {
    // Snapshot pulse widths at frame start (convert μs → 20-μs ticks).
    // Round to NEAREST tick (+10 before /20): plain truncation rounds both
    // pulses down in μs, which is asymmetric about the 1500 μs neutral and
    // un-mirrors a (1500+x, 1500−x) hip pair by up to a full 20 μs tick.
    leftEnd  = (g_servoLeftUs  + 10) / 20;
    rightEnd = leftEnd + (g_servoRightUs + 10) / 20;
    digitalWrite(SERVO_PIN_LEFT, HIGH);
  }
  if (tick == leftEnd) {
    digitalWrite(SERVO_PIN_LEFT,  LOW);
    digitalWrite(SERVO_PIN_RIGHT, HIGH);
  }
  if (tick == rightEnd) {
    digitalWrite(SERVO_PIN_RIGHT, LOW);
  }

  if (++tick >= SERVO_FRAME_TICKS) tick = 0;
}

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
float posErr       = 0.0;   // integral of velocity error = position-error state x[0]
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

// Arming — robot stays idle until RPi sends "START"
bool  running      = false;

// Timing
unsigned long prevLoopUs   = 0;
unsigned long prevSpeedMs  = 0;
unsigned long prevSerialMs = 0;
unsigned long prevReadyMs  = 0;

// ── ENCODER ISRs (Hardware Pin Change Interrupts) ─────────────
// Left Encoder ISR triggers on Pin 8 change (Port B, PCINT0)
ISR(PCINT0_vect) {
  static uint8_t lastStateL = LOW;
  uint8_t stateA = digitalRead(PIN_ENC_L_A);
  if (stateA != lastStateL) {
    lastStateL = stateA;
    if (stateA == digitalRead(PIN_ENC_L_B)) {
      encL++;
    } else {
      encL--;
    }
  }
}

// Right Encoder ISR triggers on Pin 7 change (Port D, PCINT2)
ISR(PCINT2_vect) {
  static uint8_t lastStateR = LOW;
  uint8_t stateA = digitalRead(PIN_ENC_R_A);
  if (stateA != lastStateR) {
    lastStateR = stateA;
    if (stateA == digitalRead(PIN_ENC_R_B)) {
      encR--; // Mirrored right wheel rotation direction
    } else {
      encR++;
    }
  }
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
  // Pitch (fall) axis is the IMU Y axis: rotation about Y tilts the X-Z
  // plane, so the accel angle comes from ax/az and the rate from gy.
  float accelAngle = atan2((float)ax, (float)az) * 57.2958;

  // Gyro rate (Y-axis rotation rate in deg/s).
  // If the complementary filter drifts or fights itself (tiltAngle and the
  // accel angle diverge when you tilt), flip the sign to -gy.
  tiltRate = (float)gy / 131.0;  // 131 LSB/deg/s for ±250°/s

  // Complementary filter
  tiltAngle = CF_ALPHA * (tiltAngle + tiltRate * dt) +
              (1.0 - CF_ALPHA) * accelAngle;
}

// ── MOTOR CONTROL (Yahboom YB-MNT03-v1.0 TB6612 Dual PWM Mode) ──
void setMotors(int leftPWM, int rightPWM) {
  // Left motor (Channel A) — Pin 9 (AIN1) and Pin 10 (AIN2)
  if (leftPWM > 0) {
    digitalWrite(PIN_AIN1, LOW);
    analogWrite(PIN_AIN2, constrain(leftPWM, 0, 255));
  } else if (leftPWM < 0) {
    analogWrite(PIN_AIN1, constrain(-leftPWM, 0, 255));
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
  }

  // Right motor (Channel B) — Pin 5 (BIN1) and Pin 6 (BIN2)
  if (rightPWM > 0) {
    analogWrite(PIN_BIN1, constrain(rightPWM, 0, 255));
    digitalWrite(PIN_BIN2, LOW);
  } else if (rightPWM < 0) {
    digitalWrite(PIN_BIN1, LOW);
    analogWrite(PIN_BIN2, constrain(-rightPWM, 0, 255));
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
  }
}

void stopMotors() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
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
// Full-state feedback: x = [posErr (m), velErr (m/s), pitch (rad), pitchRate (rad/s)]
// tau (N·m) = K · x  →  PWM = Ks · tau
// Signs match the lean-to-catch convention: Kp,Kd > 0; Kx,Kv < 0.
int balanceControl(float dt) {
  // Velocity setpoint from controller (-255..255 → ±1.5 m/s)
  float vTarget = (float)cmdSpeed / 255.0 * 1.5;
  float velErr  = speedAvg - vTarget;

  // Position-error state = integral of velocity error, with anti-windup clamp
  posErr += velErr * dt;
  posErr  = constrain(posErr, -0.5, 0.5);

  // Angle states in SI radians so the offline-solved K applies verbatim.
  // Sign NEGATED: the Y-axis IMU convention reads pitch opposite to the
  // wheel/motor frame, so a forward lean must command a forward (catching)
  // drive. pitch and pitchRate flip together so Kd stays damping.
  float pitch     = -(tiltAngle - balanceOffset) * 0.0174533;  // deg → rad
  float pitchRate = -tiltRate                    * 0.0174533;  // deg/s → rad/s

  // LQR state feedback → wheel torque (N·m), scaled to motor PWM
  float tau  = Kx * posErr + Kv * velErr + Kp * pitch + Kd * pitchRate;
  int output = (int)(Ks * tau);
  return constrain(output, -255, 255);
}

// ── JUMP FLIGHT DETECTION / TIMING ────────────────────────────
// The jump height sequence is coordinated directly by the high-level
// Python scripts on the RPi. The Arduino simply tracks the jumping state.
void startJump() {
  if (jumping) return;
  jumping = true;
  jumpStartMs = millis();
}

void updateJump() {
  if (!jumping) return;
  unsigned long elapsed = millis() - jumpStartMs;

  // Emulate jump duration safety window (e.g. 1.4 seconds total)
  if (elapsed >= 1400) {
    jumping = false;
  }
}

// ── SERIAL COMMAND PARSER ─────────────────────────────────────
// Protocol from RPi:
//   "START\n"                                          — arm the robot
//   "ESTOP\n"                                          — emergency stop, disarm
//   "CMD:<speed>:<turn>:<jump>\n"                      — motion command
//   "TUN:<Kx>:<Kv>:<Kp>:<Kd>:<Ks>:<balanceOffset>\n"   — set LQR gains
void parseSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line == "START") {
    // Reset control state for a clean start
    posErr = 0.0;
    fallen     = false;
    jumping    = false;
    cmdSpeed   = 0;
    cmdTurn    = 0;
    cmdJump    = false;
    running    = true;
    Serial.println("RUNNING");
    return;
  }

  if (line == "ESTOP") {
    running    = false;
    jumping    = false;
    cmdJump    = false;
    posErr = 0.0;
    stopMotors();
    // Return hips to the boot/default stance on emergency stop
    cli();
    g_servoLeftUs  = SERVO_BOOT_LEFT_US;
    g_servoRightUs = SERVO_BOOT_RIGHT_US;
    sei();
    Serial.println("STOPPED");
    return;
  }

  if (line.startsWith("CMD:")) {
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
  } else if (line.startsWith("SRV:")) {
    // Parse SRV:<id>:<pulse_us>   id: 1=left, 2=right
    int idx1 = line.indexOf(':', 4);
    if (idx1 < 0) return;
    int id  = line.substring(4,    idx1).toInt();
    int us  = line.substring(idx1+1).toInt();
    us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
    cli();
    if      (id == 1) g_servoLeftUs  = (uint16_t)us;
    else if (id == 2) g_servoRightUs = (uint16_t)us;
    sei();

  } else if (line.startsWith("TUN:")) {
    // Parse TUN:<Kx>:<Kv>:<Kp>:<Kd>:<Ks>:<balanceOffset>
    int idx1 = line.indexOf(':', 4);
    int idx2 = line.indexOf(':', idx1 + 1);
    int idx3 = line.indexOf(':', idx2 + 1);
    int idx4 = line.indexOf(':', idx3 + 1);
    int idx5 = line.indexOf(':', idx4 + 1);

    if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0 || idx5 < 0) return;

    Kx = line.substring(4,      idx1).toFloat();
    Kv = line.substring(idx1+1, idx2).toFloat();
    Kp = line.substring(idx2+1, idx3).toFloat();
    Kd = line.substring(idx3+1, idx4).toFloat();
    Ks = line.substring(idx4+1, idx5).toFloat();
    balanceOffset = line.substring(idx5+1).toFloat();

    // Send confirmation back
    Serial.print("TUN_ACK:");
    Serial.print(Kx, 4); Serial.print(":");
    Serial.print(Kv, 4); Serial.print(":");
    Serial.print(Kp, 4); Serial.print(":");
    Serial.print(Kd, 4); Serial.print(":");
    Serial.print(Ks, 4); Serial.print(":");
    Serial.println(balanceOffset, 4);
  }
}

// ── SEND TELEMETRY TO RPi ─────────────────────────────────────
// Format: TEL:<tilt>:<speed_cms>:<fallen>:<jumping>:<ax_g>:<ay_g>:<az_g>:<gx_dps>:<gy_dps>:<gz_dps>
void sendTelemetry() {
  Serial.print("TEL:");
  Serial.print(tiltAngle, 1);
  Serial.print(":");
  Serial.print(speedAvg * 100, 0);          // cm/s
  Serial.print(":");
  Serial.print(fallen  ? "1" : "0");
  Serial.print(":");
  Serial.print(jumping ? "1" : "0");
  // IMU axes in physical units (±2g accel, ±250°/s gyro)
  Serial.print(":");
  Serial.print((float)ax / 16384.0, 3);    // accel X (g)
  Serial.print(":");
  Serial.print((float)ay / 16384.0, 3);    // accel Y (g)
  Serial.print(":");
  Serial.print((float)az / 16384.0, 3);    // accel Z (g)
  Serial.print(":");
  Serial.print((float)gx / 131.0, 2);      // gyro X (°/s)
  Serial.print(":");
  Serial.print((float)gy / 131.0, 2);      // gyro Y (°/s)
  Serial.print(":");
  Serial.println((float)gz / 131.0, 2);    // gyro Z (°/s)
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);  // USB serial to RPi
  Serial.setTimeout(30); // Short timeout so readStringUntil() never blocks the balance loop

  // Motor pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  stopMotors();

  // Encoder pins
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Configure Pin Change Interrupts (PCINT) for encoders
  cli();                      // Disable global interrupts while configuring
  
  // Enable PCINT for Port B (PCIE0) and Port D (PCIE2)
  PCICR |= (1 << PCIE0);      // Port B (Pins 8-13)
  PCICR |= (1 << PCIE2);      // Port D (Pins 0-7)
  
  // Mask PCINT0 specifically for Left Encoder Ch A (Pin 8 / PCINT0)
  PCMSK0 |= (1 << PCINT0);
  
  // Mask PCINT2 specifically for Right Encoder Ch A (Pin 7 / PCINT23)
  PCMSK2 |= (1 << PCINT23);
  
  sei();                      // Re-enable interrupts


  // BLS3355 hip servo PWM output pins
  pinMode(SERVO_PIN_LEFT,  OUTPUT);
  pinMode(SERVO_PIN_RIGHT, OUTPUT);
  digitalWrite(SERVO_PIN_LEFT,  LOW);
  digitalWrite(SERVO_PIN_RIGHT, LOW);

  // Timer 2 CTC mode for 50 Hz servo ISR (20 μs tick)
  // Prescaler=8, OCR2A=39 → (39+1)*8/16e6 = 20 μs per interrupt
  TCCR2A = (1 << WGM21);              // CTC mode
  TCCR2B = (1 << CS21);               // prescaler = 8
  OCR2A  = 39;
  TIMSK2 = (1 << OCIE2A);             // enable compare-match ISR

  // MPU6050
  mpuInit();
  delay(100);

  // Warm up angle filter (let it settle for 1 second)
  // Read MPU many times so complementary filter converges
  for (int i = 0; i < 200; i++) {
    mpuRead();
    float accelAngle = atan2((float)ax, (float)az) * 57.2958;  // Y-axis pitch
    tiltAngle = 0.8 * tiltAngle + 0.2 * accelAngle;
    delay(5);
  }



  // Build banner — printed once on boot/reset so you can confirm a flash
  // actually took. If you DON'T see this line, the chip has stale firmware.
  Serial.println("FW: y-axis-pitch, sign-fixed (rate=gy, angle=atan2(ax,az))");
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

  // ── TEMPORARY: hip raise/lower test ─────────────────────────
  // Smoothly sweeps both hips ±5° about the standing stance so we can
  // watch the body rise and lower and confirm the servos carry the
  // load before switching to LQR. Wheels stay off; serial/LQR skipped.
#if SERVO_TEST_MODE
  float ph    = (millis() % SERVO_TEST_PERIOD_MS) / (float)SERVO_TEST_PERIOD_MS;
  float delta = SERVO_TEST_AMP_US * sin(ph * 2.0 * PI);
  // Hips are mirror-mounted: an EQUAL pulse spins both servos the same way,
  // lifting one leg and dropping the other (body rocks side-to-side).
  // Mirror the right pulse about the stance (left +delta, right -delta) so
  // the servos counter-rotate and both legs raise/lower the body together.
  // If the body still moves the wrong way, swap the +/- signs below.
  uint16_t leftPulse  = (uint16_t)constrain(SERVO_TEST_CENTER_US + delta, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t rightPulse = (uint16_t)constrain(SERVO_TEST_CENTER_US - delta, SERVO_MIN_US, SERVO_MAX_US);
  cli();
  g_servoLeftUs  = leftPulse;
  g_servoRightUs = rightPulse;
  sei();
  stopMotors();   // keep wheels off — hips only
  return;         // skip serial + balance loop while testing
#endif

  // Always parse serial so START / ESTOP are responsive
  parseSerial();

  // Hold idle until armed by RPi. Motors stay OFF, but we still read the IMU
  // and stream telemetry so the calibration / IMU-monitor tools work safely
  // while DISARMED (no wheels driving). Speed state is kept fresh so arming
  // doesn't produce a stale velocity spike.
  if (!running) {
    stopMotors();
    mpuRead();
    updateAngle(dt);
    updateSpeed(dt);
    if ((millis() - prevSerialMs) >= 50) {
      sendTelemetry();
      prevSerialMs = millis();
    }
    if ((millis() - prevReadyMs) >= 1000) {
      Serial.println("READY");
      prevReadyMs = millis();
    }
    return;
  }

  // 1. Read IMU and update angle
  mpuRead();
  updateAngle(dt);

  // 2. Update wheel speed
  updateSpeed(dt);

  // 3. Check for fall
  if (abs(tiltAngle - balanceOffset) > FALL_ANGLE) {
    fallen = true;
    stopMotors();
    posErr = 0.0;
  } else {
    fallen = false;
  }

  // 4. Update jump state machine
  updateJump();

  // 5. Compute and apply motor control (only if not fallen)
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

  // 6. Send telemetry every 50ms
  if ((millis() - prevSerialMs) >= 50) {
    sendTelemetry();
    prevSerialMs = millis();
  }
}
