// LINE FOLLOWING CAR
// Weighting: (-8, -4, -2, -1, +1, +2, +4, +8) / 4
//
// ASSUMPTIONS:
// 1. Calibration values came from RC discharge times in microseconds.
// 2. A longer discharge time means the sensor is over the line.
// 3. Both direction pins LOW drive the car forward.
// 4. Negative calibrated position means line left; positive means right.

// ---------------- Motor pins ----------------

const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ---------------- Sensor pins ----------------

// Ordered from left to right.
const int photoPins[8] = {
  65, 48, 64, 47, 52, 68, 53, 69
};

const float photoWeights[8] = {
  -8.0f / 4.0f,
  -4.0f / 4.0f,
  -2.0f / 4.0f,
  -1.0f / 4.0f,
   1.0f / 4.0f,
   2.0f / 4.0f,
   4.0f / 4.0f,
   8.0f / 4.0f
};

// ---------------- IR LEDs ----------------

const int IR_LED_odd  = 45;
const int IR_LED_even = 61;

// ---------------- Sensor settings ----------------

const unsigned long chargeTimeUs = 10;

// Must exceed the longest individual sensor discharge time.
// 10000 us is a starting setting, not supplied by your calibration.
const unsigned long sensorTimeoutUs = 10000;

// Starting threshold based on the original code's 2 ms read.
// Check this against individual readings on the line and background.
const unsigned long lineThresholdUs = 2000;

// ---------------- Driving settings ----------------

// Assumes the default 8-bit PWM range.
const int PWM_MAX = 255;

const int baseSpeed    = 65;
const int turnSpeed    = 85;
const int searchSpeed  = 55;

const unsigned long timeToTurnMs = 700;

// Set to -1 if the car steers away from the line.
// With +1, positive position makes the left wheel faster.
const int steeringSign = 1;


// Calibration position is divided by 32 before applying these.
const float kP = 50.0f;
const float kD = 150.0f;

const unsigned long controlPeriodUs = 5000;

//End-marker settings 

const int markerSensorCount = 6;

// Require a sustained wide line before counting it.
const unsigned long markerConfirmMs = 25;

// Require the wide line to clear before another can count.
const unsigned long markerClearMs = 100;

const int calibrationCount = 17;

const float calibratedPosition[calibrationCount] = {
  -32, -28, -24, -20, -16, -12, -8, -4,
    0,
    4,   8,  12,  16,  20,  24, 28, 32
};

const float calibratedReading[calibrationCount] = {
  -2226.719199f,
  -1560.40039f,
  -1128.548502
-725.1264949f,
-582.3227173f,
-450.0611413f,
-304.1680368f,
-183.219619f,
83.91175667f,
298.6762051f,
401.1685668f,
607.476123f,
727.3684738f,
1090.597989f,
1388.940946f,
2301.324419f,
2122.495693,
};

//State

unsigned long sensorReading[8];

float previousError = 0.0f;
float lastSeenError = 0.0f;

bool havePreviousError = false;
bool stopped = false;

int lineCount = 0;

bool markerArmed = true;
bool markerTiming = false;
bool clearTiming = false;

unsigned long markerStartedMs = 0;
unsigned long clearStartedMs = 0;
unsigned long previousControlUs = 0;

// Motor helpers

void setForwardSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  0, PWM_MAX);
  rightSpeed = constrain(rightSpeed, 0, PWM_MAX);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  analogWrite(left_pwm_pin, leftSpeed);
  analogWrite(right_pwm_pin, rightSpeed);
}

void stopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void turnAround() {
  stopMotors();

  // Reverse the left motor and drive the right motor forward.
  digitalWrite(left_dir_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);

  analogWrite(left_pwm_pin, turnSpeed);
  analogWrite(right_pwm_pin, turnSpeed);

  delay(timeToTurnMs);

  stopMotors();

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  havePreviousError = false;
  lastSeenError = 0.0f;
}

// ---------------- Sensor reading ----------------

void readSensors() {
  bool finished[8];

  // Charge all eight sensor capacitors.
  for (int i = 0; i < 8; i++) {
    pinMode(photoPins[i], OUTPUT);
    digitalWrite(photoPins[i], HIGH);

    sensorReading[i] = sensorTimeoutUs;
    finished[i] = false;
  }

  delayMicroseconds(chargeTimeUs);

  // Release the pins and disable internal pull-ups.
  for (int i = 0; i < 8; i++) {
    pinMode(photoPins[i], INPUT);
    digitalWrite(photoPins[i], LOW);
  }

  unsigned long startedUs = micros();
  int remaining = 8;

  // Measure how long each pin stays HIGH.
  while (remaining > 0) {
    unsigned long elapsedUs = micros() - startedUs;

    if (elapsedUs >= sensorTimeoutUs) {
      break;
    }

    for (int i = 0; i < 8; i++) {
      if (!finished[i] && digitalRead(photoPins[i]) == LOW) {
        sensorReading[i] = micros() - startedUs;
        finished[i] = true;
        remaining--;
      }
    }
  }
}

float getWeightedReading() {
  float result = 0.0f;

  for (int i = 0; i < 8; i++) {
    result += photoWeights[i] * sensorReading[i];
  }

  return result;
}

int getLineSensorCount() {
  int count = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorReading[i] >= lineThresholdUs) {
      count++;
    }
  }

  return count;
}

// Convert weighted reading to position using linear interpolation.
float readingToPosition(float reading) {
  if (reading <= calibratedReading[0]) {
    return calibratedPosition[0];
  }

  if (reading >= calibratedReading[calibrationCount - 1]) {
    return calibratedPosition[calibrationCount - 1];
  }

  for (int i = 0; i < calibrationCount - 1; i++) {
    if (reading <= calibratedReading[i + 1]) {
      float fraction =
        (reading - calibratedReading[i]) /
        (calibratedReading[i + 1] - calibratedReading[i]);

      return calibratedPosition[i] +
        fraction *
        (calibratedPosition[i + 1] - calibratedPosition[i]);
    }
  }

  return 0.0f;
}

// End-marker detection

// Returns true if this loop should skip normal steering.
bool handleMarker(int sensorCount) {
  unsigned long nowMs = millis();

  if (sensorCount >= markerSensorCount) {
    clearTiming = false;

    if (!markerArmed) {
      // Continue through the marker without counting it again.
      setForwardSpeeds(baseSpeed, baseSpeed);
      havePreviousError = false;
      return true;
    }

    if (!markerTiming) {
      markerTiming = true;
      markerStartedMs = nowMs;
    }

    if (nowMs - markerStartedMs >= markerConfirmMs) {
      lineCount++;

      markerArmed = false;
      markerTiming = false;

      if (lineCount == 1) {
        turnAround();
      } else {
        stopped = true;
        stopMotors();
      }

      return true;
    }

    setForwardSpeeds(baseSpeed, baseSpeed);
    havePreviousError = false;
    return true;
  }

  markerTiming = false;

  if (!markerArmed) {
    if (!clearTiming) {
      clearTiming = true;
      clearStartedMs = nowMs;
    }

    if (nowMs - clearStartedMs >= markerClearMs) {
      markerArmed = true;
      clearTiming = false;
    }
  }

  return false;
}

// Setup

void setup() {
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  stopMotors();

  pinMode(IR_LED_odd, OUTPUT);
  pinMode(IR_LED_even, OUTPUT);

  digitalWrite(IR_LED_odd, HIGH);
  digitalWrite(IR_LED_even, HIGH);
}

// Main loop
void loop() {
  if (stopped) {
    stopMotors();
    return;
  }

  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - previousControlUs;

  if (elapsedUs < controlPeriodUs) {
    return;
  }

  previousControlUs = nowUs;

  readSensors();

  int sensorCount = getLineSensorCount();

  if (handleMarker(sensorCount)) {
    return;
  }

  // Lost the line: turn toward the last known line position.
  if (sensorCount == 0) {
    if (lastSeenError > 0.0f) {
      if (steeringSign > 0) {
        setForwardSpeeds(searchSpeed, 0);
      } else {
        setForwardSpeeds(0, searchSpeed);
      }
    } else if (lastSeenError < 0.0f) {
      if (steeringSign > 0) {
        setForwardSpeeds(0, searchSpeed);
      } else {
        setForwardSpeeds(searchSpeed, 0);
      }
    } else {
      // No previous direction is available.
      stopMotors();
    }

    havePreviousError = false;
    return;
  }

  float weightedReading = getWeightedReading();
  float position = readingToPosition(weightedReading);

  // Normalize calibrated position to approximately -1 ... +1.
  // The measured centre offset is already handled by interpolation.
  float error = position / 32.0f;

  float errorChange = 0.0f;

  if (havePreviousError) {
    // Scale to a nominal 5 ms update interval.
    errorChange = (error - previousError) *
      ((float)controlPeriodUs / (float)elapsedUs);
  }

  float correction =
    steeringSign * (kP * error + kD * errorChange);

  correction = constrain(
    correction,
    -(float)baseSpeed,
     (float)baseSpeed
  );

  int leftSpeed  = (int)(baseSpeed + correction);
  int rightSpeed = (int)(baseSpeed - correction);

  setForwardSpeeds(leftSpeed, rightSpeed);

  previousError = error;
  havePreviousError = true;

  if (error > 0.02f || error < -0.02f) {
    lastSeenError = error;
  }
}
