// SIMPLE CALIBRATED LINE FOLLOWER

// ---------------- Motor pins ----------------

const int leftSleep = 31;
const int leftDir   = 29;
const int leftPWM   = 40;

const int rightSleep = 11;
const int rightDir   = 30;
const int rightPWM   = 39;

// ---------------- Sensor pins ----------------

// Left to right
const int sensorPins[8] = {
  65, 48, 64, 47, 52, 68, 53, 69
};

// (-8, -4, -2, -1, +1, +2, +4, +8) / 4
const float weights[8] = {
  -2.0f, -1.0f, -0.5f, -0.25f,
   0.25f, 0.5f, 1.0f, 2.0f
};

const int oddLED  = 45;
const int evenLED = 61;

// ---------------- Driving settings ----------------

const int baseSpeed = 65;

// Motor correction per unit of calibrated position.
// At position +20, correction = 40.
const float steeringGain = 2.0f;

// Change to -1 if the car steers away from the line.
const int steeringSign = 1;

const unsigned long timeoutUs = 10000;

// ---------------- Your calibration ----------------

const int calibrationCount = 16;

const float positions[calibrationCount] = {
  -32, -28, -24, -20, -16, -12, -8, -4,
    0,   4,   8,  12,  16,  20, 24, 28
};

const float calibration[calibrationCount] = {
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

unsigned long readings[8];

// ---------------- Setup ----------------

void setup() {
  Serial.begin(115200);

  pinMode(leftSleep, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftPWM, OUTPUT);

  pinMode(rightSleep, OUTPUT);
  pinMode(rightDir, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, 0);

  digitalWrite(leftDir, LOW);
  digitalWrite(rightDir, LOW);

  digitalWrite(leftSleep, HIGH);
  digitalWrite(rightSleep, HIGH);

  pinMode(oddLED, OUTPUT);
  pinMode(evenLED, OUTPUT);

  digitalWrite(oddLED, HIGH);
  digitalWrite(evenLED, HIGH);
}

// ---------------- Read sensors ----------------

void readSensors() {
  bool finished[8];

  // Charge all sensors.
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);

    readings[i] = timeoutUs;
    finished[i] = false;
  }

  delayMicroseconds(10);

  // Release sensors and disable pull-ups.
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
    digitalWrite(sensorPins[i], LOW);
  }

  unsigned long start = micros();
  int remaining = 8;

  // Measure how long each sensor stays HIGH.
  while (remaining > 0 && micros() - start < timeoutUs) {
    for (int i = 0; i < 8; i++) {
      if (!finished[i] && digitalRead(sensorPins[i]) == LOW) {
        readings[i] = micros() - start;
        finished[i] = true;
        remaining--;
      }
    }
  }
}

// ---------------- Calibration lookup ----------------

float getPosition(float weightedReading) {
  if (weightedReading <= calibration[0]) {
    return positions[0];
  }

  if (weightedReading >= calibration[calibrationCount - 1]) {
    return positions[calibrationCount - 1];
  }

  for (int i = 0; i < calibrationCount - 1; i++) {
    if (weightedReading <= calibration[i + 1]) {
      float fraction =
        (weightedReading - calibration[i]) /
        (calibration[i + 1] - calibration[i]);

      return positions[i] +
        fraction * (positions[i + 1] - positions[i]);
    }
  }

  return 0.0f;
}

// ---------------- Follow the line ----------------

void loop() {
  readSensors();

  // Calculate the same weighted sum used for calibration.
  float weightedReading = 0.0f;

  for (int i = 0; i < 8; i++) {
    weightedReading += weights[i] * readings[i];
  }

  // Desired position is zero, so position is the error.
  float error = getPosition(weightedReading);

  float correction = steeringSign * steeringGain * error;

  correction = constrain(
    correction,
    -(float)baseSpeed,
     (float)baseSpeed
  );

  // Positive error -> turn right.
  // Negative error -> turn left.
  int leftSpeed  = (int)(baseSpeed + correction);
  int rightSpeed = (int)(baseSpeed - correction);

  analogWrite(leftPWM, leftSpeed);
  analogWrite(rightPWM, rightSpeed);

  // Print diagnostics every 500 ms.
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    Serial.print("Sensors: ");

    for (int i = 0; i < 8; i++) {
      Serial.print(readings[i]);
      Serial.print(" ");
    }

    Serial.print("| Weighted: ");
    Serial.print(weightedReading, 2);

    Serial.print(" | Error: ");
    Serial.print(error, 2);

    Serial.print(" | Left: ");
    Serial.print(leftSpeed);

    Serial.print(" | Right: ");
    Serial.println(rightSpeed);
  }

  delay(2);
}
