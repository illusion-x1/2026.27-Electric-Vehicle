#include <Arduino.h>

// ?Pin Definitions
const uint8_t motorPWM = 9;
const uint8_t motorIN1 = 6;
const uint8_t motorIN2 = 5;
const int encoderA = 3;
const int encoderB = 2;
const uint8_t buttonPin = 10;       // Testing setup: run starts when this pin reads HIGH

// ?Parameters
float targetDistance = 9.0;
float bottleLineDistance = 2.0;
float targetTime = 20.0;
float holdDelay = 0.0;

// ?Parameters: Vehicle
const float ticksPerMeter = 9000;   // Calibrated encoder ticks per meter
const float vehicleLength = 0.5; // Measurement Point center to bottle catcher center (m)
const float bottleRadius = 0.035;   // Water Bottle diameter is 6 - 7 cm, so radius 3.0 - 3.5 cm (m)
const float bottleMargin = 0.005;   // Extra distance for the bottle rear edge past the line (m)
const int encoderSign = -1;          // Use -1 if position counts down when driving forward

// ?Parameters: Tuning
const int fullPower = 255;          // Cruise power
const int creepPower = 90;          // Power for the end of the taper and for creep pulses
const float creepZone = 0.03;       // Taper ends and creep pulses begin this far from the goal (m)
const float tolerance = 0.003;      // Close enough to the goal (m)

// ?Parameters: Motion Profile
const float topSpeed = 1.0;         // Measured speed at fullPower (m / s), see the peak speed printout
const float decelRate = 1.2;        // Planned slowdown (m / s^2), lower = earlier and gentler braking
const float finishSpeed = 0.08;     // Speed the profile slows to near the goal (m / s)
const float speedGain = 400;        // Power added per m / s of speed error, higher = stronger braking
const int brakeMax = 200;           // Max reverse power used for active braking
const float slideAllowance = 0.05;  // Forward leg ends this early because the bottle slides (m)
const int slewPerSample = 30;       // Max drive power increase per sample, higher = harsher start
const unsigned long sampleMs = 15;  // Speed measurement and control interval

const int brakePower = 100;         // Brake pulse power after the taper
const int brakeMs = 0;              // Brake pulse length, 0 for no brake (profile handles braking now)
const int pulseMs = 40;             // Creep pulse length, long enough to be visible
const int settleMs = 150;           // Pause after each pulse before checking position
const unsigned long movementTimeoutMs = 30000;

const float fineTuneGuard = 0.5;

// ?Global Variables
volatile long encoderPosition = 0;
unsigned long runStart;
float worldSpeed = 0;               // Signed speed carried between legs (m / s)
float peakSpeed = 0;

enum class LedStatus { Ready, Driving, TargetReached, TimeLimit, MovementTimeout };
enum class DriveResult { TargetReached, TimeLimit, MovementTimeout };
LedStatus ledStatus = LedStatus::Ready;
unsigned long ledStatusSince;
bool runComplete = false;

// ?Functions
void readEncoder() { encoderPosition += digitalRead(encoderB) ? 1 : -1; }

long getPos() { noInterrupts(); long pos = encoderPosition; interrupts(); return pos * encoderSign; }

void setLedStatus(LedStatus status) {
  ledStatus = status;
  ledStatusSince = millis();
}

void updateStatusLed() {
  unsigned long phase = millis() - ledStatusSince;
  bool ledOn = false;

  switch (ledStatus) {
    case LedStatus::Driving:
      ledOn = (phase / 500) % 2 == 0;
      break;
    case LedStatus::TargetReached:
      phase %= 1500;
      ledOn = phase < 150 || (phase >= 300 && phase < 450);
      break;
    case LedStatus::TimeLimit:
      phase %= 1800;
      ledOn = phase < 100 || (phase >= 200 && phase < 300) || (phase >= 400 && phase < 500);
      break;
    case LedStatus::MovementTimeout:
      phase %= 2100;
      ledOn = phase < 100 || (phase >= 200 && phase < 300) ||
              (phase >= 400 && phase < 500) || (phase >= 600 && phase < 700);
      break;
    case LedStatus::Ready:
      break;
  }

  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

void waitWithStatus(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    updateStatusLed();
    delay(1);
  }
}

// dir: 1 forward, -1 reverse, 0 coast
void setMotor(int dir, int pwmValue) {
  analogWrite(motorPWM, dir == 0 ? 0 : constrain(pwmValue, 0, 255));
  digitalWrite(motorIN1, dir < 0);
  digitalWrite(motorIN2, dir > 0);
}

bool timeUp() { return (millis() - runStart) / 1000.0 >= targetTime - fineTuneGuard; }

DriveResult driveTo(float goalMeters, int dir, bool oneWay) {
  unsigned long driveStart = millis();
  long goalTicks = goalMeters * ticksPerMeter;
  long tolTicks = tolerance * ticksPerMeter;
  long creepTicks = creepZone * ticksPerMeter;
  long stopTicks = oneWay ? tolTicks : creepTicks;  // One way leg hands off without stopping
  long err;

  long lastPos = getPos();
  unsigned long lastSample = millis();
  float speed = worldSpeed * dir;  // Speed along this leg's direction
  float lastCmd = 0;
  setMotor(0, 0);

  while ((err = (goalTicks - getPos()) * dir) > stopTicks) {
    if (millis() - driveStart >= movementTimeoutMs) {
      setMotor(0, 0);
      return DriveResult::MovementTimeout;
    }

    unsigned long now = millis();
    if (now - lastSample >= sampleMs) {
      long pos = getPos();
      float dt = (now - lastSample) / 1000.0;
      float rawSpeed = (pos - lastPos) * dir / ticksPerMeter / dt;
      speed = 0.6 * speed + 0.4 * rawSpeed;
      lastPos = pos;
      lastSample = now;
      worldSpeed = speed * dir;
      if (speed > peakSpeed) peakSpeed = speed;

      float errMeters = err / ticksPerMeter;
      float speedLimit = min(topSpeed, max(finishSpeed, sqrt(2.0 * decelRate * errMeters)));
      float cmd = fullPower * speedLimit / topSpeed + speedGain * (speedLimit - speed);

      if (speed < finishSpeed && cmd > 0) cmd = max(cmd, (float)creepPower);  // Stay above motor deadband
      if (cmd > lastCmd + slewPerSample) cmd = lastCmd + slewPerSample;       // Smooth drive ramp only
      if (cmd < 0 && speed <= 0) cmd = 0;                                     // Never brake into a reverse
      cmd = constrain(cmd, -brakeMax, fullPower);
      lastCmd = cmd;

      setMotor(cmd >= 0 ? dir : -dir, abs(cmd));
    }
    updateStatusLed();
  }

  if (oneWay) return DriveResult::TargetReached;

  // Optional brake pulse, then let the vehicle settle
  if (brakeMs > 0) {
    setMotor(-dir, brakePower);
    waitWithStatus(brakeMs);
  }
  setMotor(0, 0);
  worldSpeed = 0;
  waitWithStatus(settleMs);

  // Creep pulses until within tolerance
  while (millis() - driveStart < movementTimeoutMs) {
    err = goalTicks - getPos();
    if (abs(err) <= tolTicks || timeUp()) {
      return abs(err) <= tolTicks
                 ? DriveResult::TargetReached
                 : DriveResult::TimeLimit;
    }
    setMotor(err > 0 ? 1 : -1, creepPower);
    waitWithStatus(pulseMs);
    setMotor(0, 0);
    waitWithStatus(settleMs);
  }

  setMotor(0, 0);
  return DriveResult::MovementTimeout;
}

// ?Setup
void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);
  Serial.println("Ready");
  updateStatusLed();
}

// ?Main Loop
void loop() {
  if (runComplete) {
    updateStatusLed();
    delay(10);
    return;
  }

  // Measurement Point position that puts the bottle just past the line
  float bottleGoal = targetDistance + bottleLineDistance + bottleRadius + bottleMargin - vehicleLength;

  while (digitalRead(buttonPin) != HIGH) { updateStatusLed(); delay(1);}

  runStart = millis();
  noInterrupts();
  encoderPosition = 0;
  interrupts();
  worldSpeed = 0;
  peakSpeed = 0;
  setLedStatus(LedStatus::Driving);
  updateStatusLed();

  // ?Movement Sequence
  DriveResult result = driveTo(bottleGoal - slideAllowance, 1, true);
  if (result == DriveResult::TargetReached) {
    if (holdDelay > 0) {
      setMotor(0, 0);
      worldSpeed = 0;
      waitWithStatus(holdDelay * 1000);
    }
    result = driveTo(targetDistance, -1, false);
  }

  float runTime = (millis() - runStart) / 1000.0;
  Serial.println("Run time (s): " + String(runTime));
  Serial.println("Target time (s): " + String(targetTime));
  Serial.println("Peak speed (m / s): " + String(peakSpeed));
  Serial.println("Suggested holdDelay (s): " + String(max(0.0f, holdDelay + targetTime - runTime)));

  LedStatus finalStatus = LedStatus::TimeLimit;
  if (result == DriveResult::TargetReached) finalStatus = LedStatus::TargetReached;
  if (result == DriveResult::MovementTimeout) finalStatus = LedStatus::MovementTimeout;
  setLedStatus(finalStatus);

  runComplete = true;
}