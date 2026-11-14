#include <Arduino.h>

// ?Pin Definitions
const uint8_t motorPWM = 9;
const uint8_t motorIN1 = 6;
const uint8_t motorIN2 = 5;
const int encoderA = 3;
const int encoderB = 2;
const uint8_t buttonPin = 10;       // Testing setup: run starts when this pin reads HIGH

// ?Parameters
float targetDistance = 5;
float bottleLineDistance = 2 + targetDistance + 1;  // Bottle distance from the start (target distance plus bottle distance) (m)
float targetTime = 20;
float holdDelay = 0.0;

// ?Parameters: Vehicle
const float ticksPerMeter = 9000;   // Calibrated encoder ticks per meter
const float vehicleLength = 0.8;    // Catcher circle center to measuring point center (m)
const int encoderSign = -1;

// ?Parameters: Tuning (each leg is separate, so adjust one without touching the other)
// Trim: + stops farther from the start, - stops closer to the start (m)
const float bottleTrim = -0.245;            // Bottle leg stop point shift
const int bottleBrakePower = 255;           // Reverse power for the hard brake at the bottle line (0 - 255)
const unsigned long bottleBrakeMs = 400;    // Bottle line brake duration (ms)

const float targetTrim = 0.0;               // Target leg stop point shift
const int targetBrakePower = 255;           // Reverse power for the hard brake at the target (0 - 255)
const unsigned long targetBrakeMs = 500;    // Target brake duration (ms)

// ?Parameters: Motion
const int fullPower = 255;          // Cruise power
const int slewPerSample = 30;       // Max drive power increase per sample, higher = harsher start
const unsigned long sampleMs = 15;  // Speed measurement and control interval
const unsigned long movementTimeoutMs = 30000;

// ?Global Variables
volatile long encoderPosition = 0;
unsigned long runStart;
float peakSpeed = 0;

enum class LedStatus { Ready, Driving, TargetReached, MovementTimeout };
enum class DriveResult { TargetReached, MovementTimeout };
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

// Drive at full power to the goal, then hard brake with reverse power for a fixed time
DriveResult driveTo(float goalMeters, int dir, int brakePower, unsigned long brakeMs) {
  unsigned long driveStart = millis();
  long goalTicks = goalMeters * ticksPerMeter;

  long lastPos = getPos();
  unsigned long lastSample = millis();
  float speed = 0;
  float lastCmd = 0;
  setMotor(0, 0);

  while ((goalTicks - getPos()) * dir > 0) {
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
      if (speed > peakSpeed) peakSpeed = speed;

      float cmd = min(lastCmd + slewPerSample, (float)fullPower);  // Smooth drive ramp
      lastCmd = cmd;
      setMotor(dir, cmd);
    }
    updateStatusLed();
  }

  setMotor(-dir, brakePower);
  waitWithStatus(brakeMs);
  setMotor(0, 0);
  return DriveResult::TargetReached;
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

  float bottleGoal = bottleLineDistance - vehicleLength + bottleTrim;  // Measuring point stop for the bottle leg
  float targetGoal = targetDistance + targetTrim;                      // Measuring point stop for the return leg

  while (digitalRead(buttonPin) != HIGH) { updateStatusLed(); delay(1);}

  runStart = millis();
  noInterrupts();
  encoderPosition = 0;
  interrupts();
  peakSpeed = 0;
  setLedStatus(LedStatus::Driving);
  updateStatusLed();

  // ?Movement Sequence
  DriveResult result = driveTo(bottleGoal, 1, bottleBrakePower, bottleBrakeMs);
  if (result == DriveResult::TargetReached) {
    if (holdDelay > 0) {
      setMotor(0, 0);
      waitWithStatus(holdDelay * 1000);
    }
    result = driveTo(targetGoal, -1, targetBrakePower, targetBrakeMs);
  }

  float runTime = (millis() - runStart) / 1000.0;
  Serial.println("Run time (s): " + String(runTime));
  Serial.println("Target time (s): " + String(targetTime));
  Serial.println("Peak speed (m / s): " + String(peakSpeed));
  Serial.println("Suggested holdDelay (s): " + String(max(0.0f, holdDelay + targetTime - runTime)));

  setLedStatus(result == DriveResult::MovementTimeout ? LedStatus::MovementTimeout : LedStatus::TargetReached);

  runComplete = true;
}