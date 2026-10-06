#include <Arduino.h>

// ?Pin Definitions
// Motor Pins
const uint8_t motorPWM = 6;
const uint8_t motorIN1 = 5;
const uint8_t motorIN2 = 6;

// Encoder Pins
const int encoderA = 3;
const int encoderB = 2;

// Button
const uint8_t buttonPin = 7;

// ?Competition Inputs (meters and seconds, change these for the randomization)
float targetDistance = 7.50; // Start Point to Target Point
float bottleLineDistance = 2.00; // Target Point to Bottle Line
float targetTime = 15.0;
float previousRunTime = 15.0; // Last measured run time with no start delay

// ?Vehicle Constants
const float ticksPerMeter = 1000.0; // TODO measure
const float vehicleLength = 0.30; // Measurement Point to pusher front
const float bottleLocation = 3.0; // Start Point to bottle center (States)
const float bottleDiameter = 0.065;
const float bottleOvershoot = 0.03; // How far the bottle rear edge ends past the line

// ?Movement Parameters
const int forwardDir = 1;

const float slowZone = 0.40; // Distance before bottle contact to slow down
const float grabCarry = 0.15; // Distance pushed at grabPower after contact

const int runPower = 200;
const int grabPower = 90;
const int pushPower = 150;
const int backPower = 120;
const int minPower = 60;

const float rampRate = 150; // Power per second
const float taperPerMeter = 200; // Power per meter of remaining distance

const int brakePower = 90;
const int brakeDuration = 120;

const float fineTolerance = 0.002; // Meters
const unsigned long settleTime = 300; // Milliseconds

// ?Global Variables
volatile long encoderPos = 0;

long prevTime;
long deltaTime;
float currentPower = 0;

float contactDistance;
float lineDistance;
float pushDistance;
float runDistance;
float grabDistance;
float finishDistance;
float reverseDistance;
float startDelay;

bool waitingForButton = false;

// ?Forward Declarations
void readEncoder();
long getEncoderPos();

void fwd(float meters, int power, bool brakeAtEnd);
void back(float meters, int power, bool brakeAtEnd);
void fineAdjust(float targetMeters);

void moveStraight(float meters, char direction, int power, bool brakeAtEnd);
void rampPower(float desiredPower);

void setMotor(int dir, int pwmVal);
void stopMotors();
void applyBrake(char direction);

void computeDeltaT();

// ?PID Controller
class SimplePID {
public:
  float kp, kd, ki, umax, umin;
  float eprev, eintegral;

  SimplePID() : kp(1), kd(0), ki(0), umax(255), umin(0), eprev(0.0), eintegral(0.0) {}

  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float uminIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
    umin = uminIn;
  }

  void evaluate(double value, double target, float deltaT, int &pwr, int &dir) {
    if (deltaT <= 0) deltaT = 0.001;
    double error = target - value;
    float derivative = (error - eprev) / deltaT;
    eintegral += error * deltaT;
    float output = kp * error + kd * derivative + ki * eintegral;

    pwr = constrain((int)fabs(output), umin, umax);
    dir = (output < 0) ? -1 : 1;
    eprev = error;
  }
};

SimplePID pidFine;

// ?Setup
void setup() {
  Serial.begin(9600);
  Serial.println("Vehicle Initialization");

  // Configure PID controller (power per tick)
  pidFine.setParams(0.5, 0, 0, 100, minPower);

  // Initialize Button
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);

  // Initialize Encoder
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);

  Serial.println("Initialization Successful");
  waitingForButton = true;
}

// ?Main Loop
void loop() {
  // Run profile: fast run, slow down, grab bottle, speed up, stop,
  // reverse to Target Point, fine adjust with encoder

  contactDistance = bottleLocation - (bottleDiameter / 2) - vehicleLength;
  lineDistance = targetDistance + bottleLineDistance;
  pushDistance = lineDistance + bottleOvershoot + bottleDiameter - vehicleLength;

  runDistance = max(0.0f, contactDistance - slowZone);
  grabDistance = slowZone + grabCarry;
  finishDistance = max(0.0f, pushDistance - runDistance - grabDistance);
  reverseDistance = pushDistance - targetDistance;

  startDelay = targetTime - previousRunTime;

  while (waitingForButton) {
    if (digitalRead(buttonPin) == 1) {
      waitingForButton = false;
      Serial.println("Starting Sequences");
      delay(100); // Debounce
    } else {
      delay(10);
    }
  }

  noInterrupts();
  encoderPos = 0;
  interrupts();

  unsigned long runStart = millis();

  // ?Movement Sequence
  if (startDelay > 0) delay(startDelay * 1000);

  fwd(runDistance, runPower, false);
  fwd(grabDistance, grabPower, false);
  fwd(finishDistance, pushPower, true);

  back(reverseDistance, backPower, true);
  fineAdjust(targetDistance);

  Serial.println("Run Time: " + String((millis() - runStart) / 1000.0));
  Serial.println("Encoder Distance: " + String(getEncoderPos() / ticksPerMeter));

  while (1) {Serial.println("Sequences Terminated"); delay(1000000);}
}

// ?Movement Commands
void fwd(float meters, int power, bool brakeAtEnd) {
  moveStraight(meters, 'f', power, brakeAtEnd);
}

void back(float meters, int power, bool brakeAtEnd) {
  moveStraight(meters, 'b', power, brakeAtEnd);
}

// ?Movement Implementations
void moveStraight(float meters, char direction, int power, bool brakeAtEnd) {
  long startTicks = getEncoderPos();
  long targetTicks = (long)(meters * ticksPerMeter);

  if (currentPower < minPower) currentPower = minPower;
  prevTime = millis();

  while (abs(getEncoderPos() - startTicks) < targetTicks) {
    computeDeltaT();

    long remainingTicks = targetTicks - abs(getEncoderPos() - startTicks);
    float desiredPower = power;

    if (brakeAtEnd) {
      desiredPower = min(desiredPower, minPower + (remainingTicks / ticksPerMeter) * taperPerMeter);
    }

    rampPower(desiredPower);

    if (direction == 'f') {
      setMotor(forwardDir, (int)currentPower);
    } else {
      setMotor(-forwardDir, (int)currentPower);
    }
  }

  if (brakeAtEnd) {
    applyBrake(direction);
    stopMotors();
    currentPower = 0;
    delay(100);
  }
}

void rampPower(float desiredPower) {
  float slew = rampRate * deltaTime / 1000.0;

  if (currentPower < desiredPower) {
    currentPower = min(currentPower + slew, desiredPower);
  } else {
    currentPower = max(currentPower - slew, desiredPower);
  }

  currentPower = constrain(currentPower, minPower, 255);
}

void fineAdjust(float targetMeters) {
  long targetTicks = (long)(targetMeters * ticksPerMeter);
  long toleranceTicks = (long)(fineTolerance * ticksPerMeter);
  unsigned long settleStart = 0;

  pidFine.eprev = 0;
  pidFine.eintegral = 0;
  prevTime = millis();

  while (1) {
    computeDeltaT();

    long errorTicks = targetTicks - getEncoderPos();

    if (abs(errorTicks) <= toleranceTicks) {
      stopMotors();
      if (settleStart == 0) settleStart = millis();
      if (millis() - settleStart >= settleTime) break;
    } else {
      settleStart = 0;

      int pwr, dir;
      pidFine.evaluate(getEncoderPos(), targetTicks, deltaTime / 1000.0, pwr, dir);
      setMotor(dir * forwardDir, pwr);
    }
  }

  stopMotors();
  currentPower = 0;
}

// ?Motor Control
void setMotor(int dir, int pwmVal) {
  analogWrite(motorPWM, pwmVal);

  if (dir == -1) {
    digitalWrite(motorIN1, HIGH);
    digitalWrite(motorIN2, LOW);
  } else if (dir == 1) {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, HIGH);
  } else {
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
  }
}

void stopMotors() {
  setMotor(0, 0);
}

void applyBrake(char direction) {
  if (direction == 'f') {
    setMotor(-forwardDir, brakePower);
  } else {
    setMotor(forwardDir, brakePower);
  }

  delay(brakeDuration);
}

// ?Utility Functions
void computeDeltaT() {
  long currTime = millis();
  deltaTime = currTime - prevTime;
  prevTime = currTime;
}

long getEncoderPos() {
  noInterrupts();
  long pos = encoderPos;
  interrupts();
  return pos;
}

void readEncoder() {
  int b = digitalRead(encoderB);
  encoderPos += (b > 0) ? 1 : -1;
}