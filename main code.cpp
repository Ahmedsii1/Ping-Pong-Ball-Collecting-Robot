/*
 * Robo League - competition firmware
 *
 * Board:      Tools -> Board -> ESP32 + Bluepad32 Arduino -> ESP32 Dev Module
 * Partition:  Huge APP (3MB No OTA / 1MB SPIFFS)
 *
 * Hardware:
 *   Motor rail: 3S pack, 12.6 V full charge, shared by all DC motors
 *   Logic rail: 2S pack -> buck -> 5.0 V -> ESP32 VIN and servo
 *   Driver #1 ch A -> front-left + rear-left  (6 V yellow TT, paralleled)
 *   Driver #1 ch B -> unused
 *   Driver #2 ch A -> front-right + rear-right (6 V yellow TT, paralleled)
 *   Driver #2 ch B -> roller (Jsumo 12 V, 250 rpm)
 *   Servo          -> gate, from the 5 V rail
 *   Right side polarity is mirrored in the harness - no software invert.
 *
 * Controls:
 *   R2              throttle forward   (analog)
 *   L2              throttle reverse   (analog)
 *   Left stick X    steering
 *   R1              roller forward, press again to stop
 *   L1              roller reverse, press again to stop
 *   D-pad up/down   trim roller forward duty, 200..250 in steps of 10
 *   Cross           gate to 25 deg
 *   Square          gate to 45 deg
 *   Circle          gate to 65 deg
 *   D-pad right     gate +5 deg
 *   D-pad left      gate -5 deg
 *   Gate parks at 90 deg on power-up.
 */

#include <Bluepad32.h>

// ---------------------------------------------------------------- pin map

const int PIN_STBY   = 4;    // shared standby, both drivers

const int PIN_L_PWM  = 25;   // driver #1 PWMA
const int PIN_L_IN1  = 26;   // driver #1 AIN1
const int PIN_L_IN2  = 27;   // driver #1 AIN2

const int PIN_R_PWM  = 32;   // driver #2 PWMA
const int PIN_R_IN1  = 33;   // driver #2 AIN1
const int PIN_R_IN2  = 13;   // driver #2 AIN2

const int PIN_RL_PWM = 19;   // driver #2 PWMB
const int PIN_RL_IN1 = 21;   // driver #2 BIN1
const int PIN_RL_IN2 = 22;   // driver #2 BIN2

const int PIN_SERVO  = 18;   // gate

// ---------------------------------------------------------------- ledc config

const int MOTOR_FREQ_HZ = 20000;   // above audible range
const int MOTOR_BITS    = 8;       // 0..255

const int SERVO_FREQ_HZ = 50;
const int SERVO_BITS    = 16;

/*
 * LEDC channels are paired onto shared timers: 0-1 on timer 0, 2-3 on timer 1,
 * 4-5 on timer 2. Channels sharing a timer must share a frequency. The three
 * motors all run 20 kHz so 0, 1 and 2 sit together happily. The servo needs
 * 50 Hz, so it goes on channel 4 and channel 3 is deliberately left empty.
 */
const int LEDC_CH_L   = 0;
const int LEDC_CH_R   = 1;
const int LEDC_CH_RL  = 2;
const int LEDC_CH_SVO = 4;

// ---------------------------------------------------------------- drive limits

/*
 * The rail is 12.6 V at full charge and the drive motors are 6 V parts, so
 * 255 * (6.0 / 12.6) = 121. This drifts below 6 V as the pack sags, which is
 * the safe direction to be wrong in.
 * NEVER raise this without physically lowering the rail voltage.
 */
const int DRIVE_CAP = 121;

/*
 * Steering authority as a percentage of full drive. 100 gives spin-in-place at
 * full stick. Drop toward 70 if the robot feels twitchy on the rubber matting.
 */
const int STEER_GAIN_PCT = 100;

// ---------------------------------------------------------------- roller limits

/*
 * Forward duty is trimmed live from the D-pad to compensate for pack droop.
 * At 250 on a full 12.6 V rail the motor sees 12.35 V - fine for a 12 V part,
 * and the headroom is there precisely so you can climb as the pack falls.
 */
const int ROLLER_FWD_DEFAULT = 210;
const int ROLLER_FWD_MIN     = 200;
const int ROLLER_FWD_MAX     = 250;
const int ROLLER_TRIM_STEP   = 10;

// Fixed. The D-pad trim does not touch this - reverse is a short unjam burst,
// not something that needs droop compensation.
const int ROLLER_REVERSE_DUTY = -60;

// ---------------------------------------------------------------- gate limits

const int GATE_MIN_DEG  = 25;
const int GATE_MAX_DEG  = 70;
const int GATE_STEP_DEG = 5;    // D-pad left / right

const int GATE_PRESET_CROSS  = 25;
const int GATE_PRESET_SQUARE = 45;
const int GATE_PRESET_CIRCLE = 65;

/*
 * Parked position at power-up. This sits outside the [25,70] operating band on
 * purpose - it is where the gate rests before a match. The first D-pad press
 * clamps it to 70; any face-button preset jumps it straight to its value.
 */
const int GATE_BOOT_DEG = 90;

// Servo pulse range. 500-2500 us is wider than the 1000-2000 us standard and
// gets full travel from most micro servos. Narrow to 600/2400 if yours buzzes.
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2500;

// ---------------------------------------------------------------- input shaping

/*
 * Bluepad32 reports sticks over -512..511 and the analog triggers over 0..1023.
 * A DS4 stick rarely returns to exactly zero, so anything inside the deadzone
 * counts as centred and values just outside are rescaled from zero rather than
 * jumping.
 */
const int STICK_RANGE      = 512;
const int STICK_DEADZONE   = 40;
const int TRIGGER_RANGE    = 1023;
const int TRIGGER_DEADZONE = 30;

// ---------------------------------------------------------------- rate limits

/*
 * Ramping instead of stepping. Two TT motors share one 1.2 A channel and the
 * roller is a geared 12 V motor with real inertia - a hard step spikes the
 * current and, with no bulk capacitor fitted, spikes the VM rail with it.
 *
 * At 5 counts per 10 ms tick, a full R1-to-L1 reversal (210 down to -60) takes
 * about 540 ms. Raise SLEW_STEP_ROLLER to 8 for a snappier unjam, at the cost
 * of a harder current spike through driver #2.
 */
const int           SLEW_STEP_DRIVE  = 3;    // duty counts per tick
const int           SLEW_STEP_ROLLER = 5;
const unsigned long SLEW_TICK_MS     = 10;

/*
 * The servo shares its buck with the ESP32 and has no bulk capacitor, so a
 * full-throw command would pull close to an amp within milliseconds. Two
 * degrees every 20 ms spreads that out and the buck never notices.
 */
const int           SERVO_STEP_DEG = 2;
const unsigned long SERVO_TICK_MS  = 20;

// ---------------------------------------------------------------- link failsafe

// If the controller stops reporting connected for this long, disable the
// drivers. A robot still running its last command drives itself out of
// bounds, and rule 3 stops it scoring for the rest of that instance.
const unsigned long LINK_TIMEOUT_MS = 300;

// ---------------------------------------------------------------- state

ControllerPtr myController = nullptr;

enum RollerMode { ROLLER_OFF, ROLLER_FWD, ROLLER_REV };
RollerMode rollerMode    = ROLLER_OFF;
int        rollerFwdDuty = ROLLER_FWD_DEFAULT;

int targetLeft   = 0;   // all signed
int targetRight  = 0;
int targetRoller = 0;

int actualLeft   = 0;
int actualRight  = 0;
int actualRoller = 0;

int targetGate = GATE_BOOT_DEG;
int actualGate = GATE_BOOT_DEG;

// Previous-frame button states, for rising-edge detection. Without these a
// held D-pad would trim continuously instead of one step per press.
bool prevR1 = false, prevL1 = false;
bool prevUp = false, prevDown = false, prevRight = false, prevLeft = false;
bool prevCross = false, prevSquare = false, prevCircle = false;

bool          driversEnabled = false;
unsigned long lastSlewTick   = 0;
unsigned long lastServoTick  = 0;
unsigned long lastPacketMs   = 0;

// ---------------------------------------------------------------- ledc helpers

// The ESP32 Arduino core changed the LEDC API at version 3.0. These wrappers
// keep the sketch compiling on either core.

void ledcInitPin(int pin, int channel, int freq, int bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, freq, bits);
#else
  ledcSetup(channel, freq, bits);
  ledcAttachPin(pin, channel);
#endif
}

void ledcWritePin(int pin, int channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

int stepToward(int current, int target, int maxStep) {
  if (current < target) return min(current + maxStep, target);
  if (current > target) return max(current - maxStep, target);
  return current;
}

// ---------------------------------------------------------------- motor output

/*
 * Drives one H-bridge channel. Positive duty sets IN1 high and IN2 low;
 * negative swaps them. Zero pulls both low, which coasts rather than brakes -
 * gentler on the gearboxes and it lets the robot roll to a stop rather than
 * stopping dead and tipping forward.
 */
void applyChannel(int in1, int in2, int pwmPin, int channel, int signedDuty) {
  if (signedDuty > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (signedDuty < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWritePin(pwmPin, channel, abs(signedDuty));
}

void updateMotors() {
  if (millis() - lastSlewTick < SLEW_TICK_MS) return;
  lastSlewTick = millis();

  int newLeft   = stepToward(actualLeft,   targetLeft,   SLEW_STEP_DRIVE);
  int newRight  = stepToward(actualRight,  targetRight,  SLEW_STEP_DRIVE);
  int newRoller = stepToward(actualRoller, targetRoller, SLEW_STEP_ROLLER);

  if (newLeft != actualLeft) {
    actualLeft = newLeft;
    applyChannel(PIN_L_IN1, PIN_L_IN2, PIN_L_PWM, LEDC_CH_L, actualLeft);
  }
  if (newRight != actualRight) {
    actualRight = newRight;
    applyChannel(PIN_R_IN1, PIN_R_IN2, PIN_R_PWM, LEDC_CH_R, actualRight);
  }
  if (newRoller != actualRoller) {
    actualRoller = newRoller;
    applyChannel(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_PWM, LEDC_CH_RL, actualRoller);
  }
}

// ---------------------------------------------------------------- servo output

/*
 * At 50 Hz the period is 20000 us. With 16-bit resolution one count is
 * 20000/65536 = 0.305 us, so duty = pulse_us * 65536 / 20000.
 */
void servoWriteDeg(int deg) {
  deg = constrain(deg, 0, 180);
  long pulse_us = map(deg, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  uint32_t duty = (uint32_t)(pulse_us * 65536L / 20000L);
  ledcWritePin(PIN_SERVO, LEDC_CH_SVO, duty);
  actualGate = deg;
}

void updateServo() {
  if (millis() - lastServoTick < SERVO_TICK_MS) return;
  lastServoTick = millis();

  int next = stepToward(actualGate, targetGate, SERVO_STEP_DEG);
  if (next != actualGate) servoWriteDeg(next);
}

// ---------------------------------------------------------------- roller logic

void setRollerMode(RollerMode m) {
  rollerMode = m;
  switch (m) {
    case ROLLER_FWD: targetRoller = rollerFwdDuty;       break;
    case ROLLER_REV: targetRoller = ROLLER_REVERSE_DUTY; break;
    default:         targetRoller = 0;                   break;
  }
}

void trimRoller(int delta) {
  rollerFwdDuty = constrain(rollerFwdDuty + delta, ROLLER_FWD_MIN, ROLLER_FWD_MAX);
  // Takes effect immediately if the roller is already running forward.
  if (rollerMode == ROLLER_FWD) targetRoller = rollerFwdDuty;

  Serial.print(F("roller trim -> "));
  Serial.println(rollerFwdDuty);
}

// ---------------------------------------------------------------- enable / disable

void enableDrivers() {
  if (driversEnabled) return;
  digitalWrite(PIN_STBY, HIGH);
  driversEnabled = true;
}

/*
 * Failsafe. Zeroes every DC motor and writes it straight through without
 * waiting for the ramp, then pulls STBY low so the drivers cannot pass current
 * whatever the logic pins are doing. The gate is deliberately left where it is
 * - a link drop should not dump the balls you have collected.
 */
void disableDrivers() {
  rollerMode = ROLLER_OFF;
  targetLeft = targetRight = targetRoller = 0;
  actualLeft = actualRight = actualRoller = 0;

  applyChannel(PIN_L_IN1,  PIN_L_IN2,  PIN_L_PWM,  LEDC_CH_L,  0);
  applyChannel(PIN_R_IN1,  PIN_R_IN2,  PIN_R_PWM,  LEDC_CH_R,  0);
  applyChannel(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_PWM, LEDC_CH_RL, 0);

  digitalWrite(PIN_STBY, LOW);
  driversEnabled = false;
}

// ---------------------------------------------------------------- controller
// (connection handling follows the proven reference sketch)

void onConnected(ControllerPtr ctl) {
  if (myController != nullptr) return;    // one controller only

  myController = ctl;

  ControllerProperties p = ctl->getProperties();
  Serial.printf("Connected: %s  VID=0x%04x PID=0x%04x\n",
                ctl->getModelName().c_str(), p.vendor_id, p.product_id);
  Serial.printf("roller trim %d   gate %d\n", rollerFwdDuty, targetGate);

  ctl->setColorLED(0, 255, 0);                  // green = armed
  ctl->playDualRumble(0, 150, 0x60, 0x60);
  BP32.enableNewBluetoothConnections(false);    // stop scanning once paired

  prevR1 = prevL1 = false;
  prevUp = prevDown = prevRight = prevLeft = false;
  prevCross = prevSquare = prevCircle = false;

  lastPacketMs = millis();
  enableDrivers();
}

void onDisconnected(ControllerPtr ctl) {
  if (myController != ctl) return;

  myController = nullptr;
  disableDrivers();
  BP32.enableNewBluetoothConnections(true);     // accept a re-pair
  Serial.println("Controller lost - drive and roller disabled.");
}

// ---------------------------------------------------------------- input shaping

/*
 * Removes the centre deadzone and rescales what remains back to full range, so
 * the first movement past the deadzone gives a small output rather than an
 * immediate jump.
 */
int shapeAxis(int raw) {
  int magnitude = abs(raw);
  if (magnitude <= STICK_DEADZONE) return 0;

  long scaled = (long)(magnitude - STICK_DEADZONE) * STICK_RANGE
                / (STICK_RANGE - STICK_DEADZONE);
  if (scaled > STICK_RANGE) scaled = STICK_RANGE;

  return (raw < 0) ? -(int)scaled : (int)scaled;
}

int shapeTrigger(int raw) {
  if (raw <= TRIGGER_DEADZONE) return 0;
  long scaled = (long)(raw - TRIGGER_DEADZONE) * TRIGGER_RANGE
                / (TRIGGER_RANGE - TRIGGER_DEADZONE);
  return (int)min(scaled, (long)TRIGGER_RANGE);
}

// ---------------------------------------------------------------- input handling

void readController() {
  // ---- drive: R2 forward, L2 reverse, left stick X steers ----

  int fwd = shapeTrigger(myController->throttle());   // R2, 0..1023
  int rev = shapeTrigger(myController->brake());      // L2, 0..1023

  // Rescale the trigger difference onto the same +/-512 scale as the stick.
  long throttle = (long)(fwd - rev) * STICK_RANGE / TRIGGER_RANGE;
  long steer    = (long)shapeAxis(myController->axisX()) * STEER_GAIN_PCT / 100;

  long mixLeft  = throttle + steer;
  long mixRight = throttle - steer;

  /*
   * Scale both sides together when the mix overflows rather than clipping one.
   * Clipping would quietly change the turn ratio at high speed - the robot
   * would understeer exactly when it is hardest to correct for.
   */
  long peak = max(labs(mixLeft), labs(mixRight));
  if (peak > STICK_RANGE) {
    mixLeft  = mixLeft  * STICK_RANGE / peak;
    mixRight = mixRight * STICK_RANGE / peak;
  }

  targetLeft  = (int)(mixLeft  * DRIVE_CAP / STICK_RANGE);
  targetRight = (int)(mixRight * DRIVE_CAP / STICK_RANGE);

  // ---- roller: R1 / L1 toggles, D-pad up/down trims ----

  bool r1 = myController->r1();
  bool l1 = myController->l1();

  /*
   * Each button toggles its own direction. Pressing the OTHER button takes over
   * immediately rather than requiring a stop first - you reach for L1 exactly
   * when a ball has jammed, and you may well still be holding R1 at that point.
   */
  if (r1 && !prevR1) {
    setRollerMode(rollerMode == ROLLER_FWD ? ROLLER_OFF : ROLLER_FWD);
  }
  if (l1 && !prevL1) {
    setRollerMode(rollerMode == ROLLER_REV ? ROLLER_OFF : ROLLER_REV);
  }
  prevR1 = r1;
  prevL1 = l1;

  uint8_t dpad  = myController->dpad();
  bool    up    = dpad & DPAD_UP;
  bool    down  = dpad & DPAD_DOWN;
  bool    right = dpad & DPAD_RIGHT;
  bool    left  = dpad & DPAD_LEFT;

  if (up   && !prevUp)   trimRoller(+ROLLER_TRIM_STEP);
  if (down && !prevDown) trimRoller(-ROLLER_TRIM_STEP);

  // ---- gate: presets on the face buttons, D-pad left/right nudges ----

  bool cross  = myController->a();   // Cross  on a DualShock 4
  bool square = myController->x();   // Square
  bool circle = myController->b();   // Circle

  if (cross  && !prevCross)  targetGate = GATE_PRESET_CROSS;
  if (square && !prevSquare) targetGate = GATE_PRESET_SQUARE;
  if (circle && !prevCircle) targetGate = GATE_PRESET_CIRCLE;

  if (right && !prevRight)
    targetGate = constrain(targetGate + GATE_STEP_DEG, GATE_MIN_DEG, GATE_MAX_DEG);
  if (left && !prevLeft)
    targetGate = constrain(targetGate - GATE_STEP_DEG, GATE_MIN_DEG, GATE_MAX_DEG);

  prevUp = up; prevDown = down; prevRight = right; prevLeft = left;
  prevCross = cross; prevSquare = square; prevCircle = circle;
}

// ---------------------------------------------------------------- setup
// (order follows the proven reference sketch: serial, hardware, stop, BT)

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_STBY, OUTPUT);
  const int dirPins[] = { PIN_L_IN1,  PIN_L_IN2,
                          PIN_R_IN1,  PIN_R_IN2,
                          PIN_RL_IN1, PIN_RL_IN2 };
  for (int p : dirPins) pinMode(p, OUTPUT);

  ledcInitPin(PIN_L_PWM,  LEDC_CH_L,  MOTOR_FREQ_HZ, MOTOR_BITS);
  ledcInitPin(PIN_R_PWM,  LEDC_CH_R,  MOTOR_FREQ_HZ, MOTOR_BITS);
  ledcInitPin(PIN_RL_PWM, LEDC_CH_RL, MOTOR_FREQ_HZ, MOTOR_BITS);

  disableDrivers();                 // STBY stays LOW until a pad connects

  BP32.setup(&onConnected, &onDisconnected);
  BP32.enableVirtualDevice(false);
  // BP32.forgetBluetoothKeys();    // uncomment ONLY when you need to re-pair

  // Servo comes up last, after the Bluetooth stack is running. Its inrush and
  // BT start-up share the same buck with no bulk capacitor - keeping them apart
  // in time avoids a brownout during the handshake.
  delay(300);
  ledcInitPin(PIN_SERVO, LEDC_CH_SVO, SERVO_FREQ_HZ, SERVO_BITS);
  servoWriteDeg(GATE_BOOT_DEG);
  targetGate = GATE_BOOT_DEG;

  Serial.println("Robo League ready. Hold SHARE + PS to pair.");
}

// ---------------------------------------------------------------- loop
// (structure follows the proven reference sketch)

void loop() {
  BP32.update();

  if (myController && myController->isConnected()) {
    lastPacketMs = millis();
    readController();
  }

  // Link failsafe - fires when the pad stops reporting connected.
  if (millis() - lastPacketMs > LINK_TIMEOUT_MS) disableDrivers();

  updateMotors();
  updateServo();

  delay(10);      // ~100 Hz. Do NOT remove - BTstack needs CPU time
}