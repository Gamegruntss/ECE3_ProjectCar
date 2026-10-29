#include <ECE3.h>

// ---------------- Motor pins ----------------

const int leftSleep = 31;
const int leftDir   = 29;
const int leftPWM   = 40;

const int rightSleep = 11;
const int rightDir   = 30;
const int rightPWM   = 39;

// ---------------- Driving settings ----------------

const int baseSpeed = 65;

// PWM correction per unit of calibrated error.
const float kP = 2.0f;

// Change to -1 if the car turns away from the line.
const int steeringSign = 1;

// ---------------- Sensors ----------------

uint16_t sensorValues[8];
float averagedSensors[8];

// Keep the SAME sensor order returned during calibration.
const float weights[8] = {
  -8.0f / 4.0f,
  -4.0f / 4.0f,
  -2.0f / 4.0f,
  -1.0f / 4.0f,
   1.0f / 4.0f,
   2.0f / 4.0f,
   4.0f / 4.0f,
   8.0f / 4.0f
};

// ---------------- Your calibration ----------------

// Use the increasing section of your calibration.
// The +32 reading folds back below the +28 reading.
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

// ---------------- Setup ----------------

void setup() {
  // Use the SAME initialization as your calibration sketch.
  ECE3_Init();

  Serial.begin(9600);

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

  delay(1000);
}

// ---------------- Read sensors ----------------

void readAveragedSensors() {
  uint32_t sums[8] = {0};

  // Same five-sample averaging as your calibration.
  for (int sample = 0; sample < 5; sample++) {
    ECE3_read_IR(sensorValues);

    for (int i = 0; i < 8; i++) {
      sums[i] += sensorValues[i];
    }
  }

  for (int i = 0; i < 8; i++) {
    averagedSensors[i] = sums[i] / 5.0f;
  }
}

// ---------------- Convert reading to error ----------------

float getCalibratedError(float reading) {
  if (reading <= calibration[0]) {
    return positions[0];
  }

  if (reading >= calibration[calibrationCount - 1]) {
    return positions[calibrationCount - 1];
  }

  for (int i = 0; i < calibrationCount - 1; i++) {
    if (reading <= calibration[i + 1]) {
      float fraction =
        (reading - calibration[i]) /
        (calibration[i + 1] - calibration[i]);

      return positions[i] +
        fraction * (positions[i + 1] - positions[i]);
    }
  }

  return 0.0f;
}

// ---------------- Main loop ----------------

void loop() {
  readAveragedSensors();

  // Apply your 8-4-2-1 weighting.
  float weightedReading = 0.0f;

  for (int i = 0; i < 8; i++) {
    weightedReading += weights[i] * averagedSensors[i];
  }

  // Your calibration maps weighted reading to position error.
  float error = getCalibratedError(weightedReading);

  // Simple proportional steering.
  float correction = steeringSign * kP * error;

  correction = constrain(
    correction,
    -(float)baseSpeed,
     (float)baseSpeed
  );

  int leftSpeed  = (int)(baseSpeed + correction);
  int rightSpeed = (int)(baseSpeed - correction);

  analogWrite(leftPWM, leftSpeed);
  analogWrite(rightPWM, rightSpeed);

  // Diagnostics every half second.
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    Serial.print("Sensors: ");

    for (int i = 0; i < 8; i++) {
      Serial.print(averagedSensors[i], 0);
      Serial.print(" ");
    }

    Serial.print("| Weighted: ");
    Serial.print(weightedReading, 1);

    Serial.print(" | Error: ");
    Serial.print(error, 1);

    Serial.print(" | Motors: ");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpeed);
  }
}
