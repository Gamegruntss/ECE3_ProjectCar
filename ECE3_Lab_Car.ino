// LINE FOLLOWING CAR — DIAGNOSTIC VERSION
// Weighting: (-8, -4, -2, -1, +1, +2, +4, +8) / 4
//
// Assumes:
// - Calibration uses RC discharge times in microseconds.
// - Longer discharge time means line detected.
// - Both motor direction pins LOW means forward.
//
// If no line has been detected yet, drive forward to acquire it.

// ---------------- Motor pins ----------------

const int left_nslp_pin  = 31;
const int left_dir_pin   = 29;
const int left_pwm_pin   = 40;

const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// ---------------- Sensors ----------------

// Left to right.
const int photoPins[8] = {
  65, 48, 64, 47, 52, 68, 53, 69
};

const float photoWeights[8] = {
  -2.0f, -1.0f, -0.5f, -0.25f,
   0.25f, 0.5f, 1.0f, 2.0f
};

const int IR_LED_odd  = 45;
const int IR_LED_even = 61;

const unsigned long chargeTimeUs = 10;
const unsigned long sensorTimeoutUs = 10000;

// Still needs verification against individual sensor readings.
const unsigned long lineThresholdUs = 2000;

// ---------------- Driving ----------------

const int PWM_MAX = 255;

const int baseSpeed   = 65;
const int turnSpeed   = 85;
const int searchSpeed = 55;

const unsigned long timeToTurnMs = 700;

// Change to -1 if steering is reversed.
const int steeringSign = 1;

const float kP = 50.0f;
const float kD = 150.0f;

const unsigned long controlPeriodUs = 5000;

// ---------------- End markers ----------------

const int markerSensorCount = 6;

const unsigned long markerConfirmMs = 25;
const unsigned long markerClearMs = 100;

// ---------------- Calibration ----------------

// Missing comma fixed.
// +32 excluded because its reading decreases relative to +28.
const int calibrationCount = 16;

const float calibratedPosition[calibrationCount] = {
  -32, -28, -24, -20, -16, -12, -8, -4,
    0,   4,   8,  12,  16,  20, 24, 28
};

const float calibratedReading[calibrationCount] = {
  -2226.719199f,
  -1560.400390f,
  -1128.548502f,
   -725.1264949f,
   -582.3227173f,
   -450.0611413f,
   -304.1680368f,
   -183.2196190f,
     83.91175667f,
    298.6762051f,
    401.1685668f,
    607.4761230f,
    727.3684738f,
   1090.597989f,
   1388.940946f,
   2301.324419f
};

// ---------------- State ----------------

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

// ---------------- Diagnostics ----------------

int commandedLeftPWM = 0;
int commandedRightPWM = 0;

float debugWeighted = 0.0f;
float debugPosition = 0.0f;
float debugError = 0.0f;
float debugCorrection = 0.0f;

int debugSensorCount = 0;
unsigned long lastPrintMs = 0;

// ---------------- Motor helpers ----------------

void setForwardSpeeds(int leftSpeed, int rightSpeed) {
  commandedLeftPWM = constrain(leftSpeed, 0, PWM_MAX);
  commandedRightPWM = constrain(rightSpeed, 0, PWM_MAX);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  analogWrite(left_pwm_pin, commandedLeftPWM);
  analogWrite(right_pwm_pin, commandedRightPWM);
}

void stopMotors() {
  commandedLeftPWM = 0;
  commandedRightPWM = 0;

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

void printDiagnostics(const char* state) {
  unsigned long nowMs = millis();

  if (nowMs - lastPrintMs < 500) {
    return;
  }

  lastPrintMs = nowMs;

  Serial.print("t=");
  Serial.print(nowMs);

  Serial.print(" | STATE=");
  Serial.print(state);

  Serial.print(" | sensors_us=[");

  int timeoutCount = 0;

  for (int i = 0; i < 8; i++) {
    Serial.print(sensorReading[i]);

    if (i < 7) {
      Serial.print(",");
    }

    if (sensorReading[i] >= sensorTimeoutUs) {
      timeoutCount++;
    }
  }

  Serial.print("] | lineBits=");

  for (int i = 0; i < 8; i++) {
    Serial.print(sensorReading[i] >= lineThresholdUs ? 1 : 0);
  }

  Serial.print(" | count=");
  Serial.print(debugSensorCount);

  Serial.print(" | timeouts=");
  Serial.print(timeoutCount);

  Serial.print(" | weighted=");
  Serial.print(debugWeighted, 2);

  Serial.print(" | position=");
  Serial.print(debugPosition, 2);

  Serial.print(" | error=");
  Serial.print(debugError, 3);

  Serial.print(" | correction=");
  Serial.print(debugCorrection, 2);

  Serial.print(" | PWM_L=");
  Serial.print(commandedLeftPWM);

  Serial.print(" | PWM_R=");
  Serial.print(commandedRightPWM);

  Serial.print(" | markers=");
  Serial.print(lineCount);

  Serial.print(" | armed=");
  Serial.println(markerArmed ? 1 : 0);
}

// ---------------- Turnaround ----------------

void turnAround() {
  Serial.println("EVENT: First marker. Starting turnaround.");

  stopMotors();

  digitalWrite(left_dir_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);

  commandedLeftPWM = turnSpeed;
  commandedRightPWM = turnSpeed;

  analogWrite(left_pwm_pin, commandedLeftPWM);
  analogWrite(right_pwm_pin, commandedRightPWM);

  Serial.print("TURN: left reverse, right forward, PWM=");
  Serial.println(turnSpeed);

  delay(timeToTurnMs);

  stopMotors();

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  havePreviousError = false;
  lastSeenError = 0.0f;

  Serial.println("EVENT: Turnaround finished.");
}

// ---------------- Read sensors ----------------

void readSensors() {
  bool finished[8];

  for (int i = 0; i < 8; i++) {
    pinMode(photoPins[i], OUTPUT);
    digitalWrite(photoPins[i], HIGH);

    sensorReading[i] = sensorTimeoutUs;
    finished[i] = false;
  }

  delayMicroseconds(chargeTimeUs);

  for (int i = 0; i < 8; i++) {
    pinMode(photoPins[i], INPUT);
    digitalWrite(photoPins[i], LOW);
  }

  unsigned long startedUs = micros();
  int remaining = 8;

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

// ---------------- Calibration lookup ----------------

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

// ---------------- Marker handling ----------------

bool handleMarker(int sensorCount) {
  unsigned long nowMs = millis();

  if (sensorCount >= markerSensorCount) {
    clearTiming = false;
    havePreviousError = false;

    if (!markerArmed) {
      setForwardSpeeds(baseSpeed, baseSpeed);
      printDiagnostics("CROSSING_MARKER");
      return true;
    }

    if (!markerTiming) {
      markerTiming = true;
      markerStartedMs = nowMs;

      Serial.println("EVENT: Wide line detected; checking marker.");
    }

    if (nowMs - markerStartedMs >= markerConfirmMs) {
      lineCount++;

      markerArmed = false;
      markerTiming = false;

      Serial.print("EVENT: Confirmed marker ");
      Serial.println(lineCount);

      if (lineCount == 1) {
        turnAround();
      } else {
        stopped = true;
        stopMotors();

        Serial.println("EVENT: Second marker. Stopped until reset.");
      }

      return true;
    }

    setForwardSpeeds(baseSpeed, baseSpeed);
    printDiagnostics("CONFIRMING_MARKER");
    return true;
  }

  if (markerTiming) {
    Serial.println("EVENT: Wide reading cleared before confirmation.");
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

      Serial.println("EVENT: Marker detection rearmed.");
    }
  }

  return false;
}

// ---------------- Setup ----------------

void setup() {
  Serial.begin(115200);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  stopMotors();

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(IR_LED_odd, OUTPUT);
  pinMode(IR_LED_even, OUTPUT);

  digitalWrite(IR_LED_odd, HIGH);
  digitalWrite(IR_LED_even, HIGH);

  Serial.println();
  Serial.println("BOOT: Diagnostic line follower started.");
  Serial.println("CONFIG: Motor sleep pins commanded HIGH.");
  Serial.println("CONFIG: Direction pins LOW = assumed forward.");
  Serial.println("CONFIG: IR LED pins commanded HIGH.");
  Serial.println("CONFIG: No initial line -> drive forward.");

  Serial.print("CONFIG: Line threshold, us = ");
  Serial.println(lineThresholdUs);

  Serial.print("CONFIG: Sensor timeout, us = ");
  Serial.println(sensorTimeoutUs);

  Serial.print("CONFIG: Base PWM = ");
  Serial.println(baseSpeed);
}

// ---------------- Main loop ----------------

void loop() {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - previousControlUs;

  if (elapsedUs < controlPeriodUs) {
    return;
  }

  previousControlUs = nowUs;

  // Keep sensor diagnostics live even after the finish stop.
  readSensors();

  debugSensorCount = getLineSensorCount();
  debugWeighted = getWeightedReading();
  debugPosition = readingToPosition(debugWeighted);
  debugError = debugPosition / 32.0f;
  debugCorrection = 0.0f;

  if (stopped) {
    stopMotors();
    printDiagnostics("STOPPED_SECOND_MARKER");
    return;
  }

  if (handleMarker(debugSensorCount)) {
    return;
  }

  // ---------------- No line detected ----------------

  if (debugSensorCount == 0) {
    havePreviousError = false;

    if (lastSeenError > 0.0f) {
      if (steeringSign > 0) {
        setForwardSpeeds(searchSpeed, 0);
      } else {
        setForwardSpeeds(0, searchSpeed);
      }

      printDiagnostics("LOST_SEARCH_POSITIVE");

    } else if (lastSeenError < 0.0f) {
      if (steeringSign > 0) {
        setForwardSpeeds(0, searchSpeed);
      } else {
        setForwardSpeeds(searchSpeed, 0);
      }

      printDiagnostics("LOST_SEARCH_NEGATIVE");

    } else {
      // Changed: move forward instead of remaining stopped.
      setForwardSpeeds(baseSpeed, baseSpeed);
      printDiagnostics("SEEKING_LINE_FORWARD");
    }

    return;
  }

  // ---------------- Follow detected line ----------------

  float error = debugError;
  float errorChange = 0.0f;

  if (havePreviousError) {
    errorChange =
      (error - previousError) *
      ((float)controlPeriodUs / (float)elapsedUs);
  }

  float correction =
    steeringSign * (kP * error + kD * errorChange);

  correction = constrain(
    correction,
    -(float)baseSpeed,
     (float)baseSpeed
  );

  debugCorrection = correction;

  int leftSpeed = (int)(baseSpeed + correction);
  int rightSpeed = (int)(baseSpeed - correction);

  setForwardSpeeds(leftSpeed, rightSpeed);

  previousError = error;
  havePreviousError = true;

  if (error > 0.02f || error < -0.02f) {
    lastSeenError = error;
  }

  printDiagnostics("FOLLOWING");
}
