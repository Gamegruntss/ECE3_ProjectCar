#include <ECE3.h>
#include <math.h>

const int leftSleep = 31;
const int leftDir = 29;
const int leftPWM = 40;
const int rightSleep = 11;
const int rightDir = 30;
const int rightPWM = 39;

// Start with the steering sign that worked on the straight track.
const int steeringSign = -1;
const int baseSpeed = 50;
const int cornerSpeed = 32;
const int maxPWM = 100;
const float kP = 2.0f;
const float kD = 0.04f;  // Error rate is measured per second.
const float derivativeFilterSeconds = 0.035f; 

// Calibrated S1 through S8, in ECE3_read_IR order.
const float sensorMin[8] = {
  749.75f, 646.25f, 669.25f, 549.75f, 645.75f, 634.50f, 697.25f, 782.50f
};
const float sensorSpan[8] = {
  1750.25f, 1840.00f, 1830.75f, 947.50f, 1450.50f, 1851.50f, 1802.75f, 1717.50f
};
// Relative positions across the sensor bar, in calibration position units.
const float sensorPosition[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
const float centerOffset = 1.04f;  // Centroid measured on the sheet's zero row.
const float noiseFloor = 0.10f;
const float lineThreshold = 0.20f;

const int searchSpeed = 45;
const unsigned long searchLimitMs = 700;
const bool diagnostics = false;  // Set true to print peak, error and motor commands.

uint16_t rawSensors[8];
float lastError = 0.0f;
float filteredRate = 0.0f;
int lastLineSide = 0;
bool trackedLastFrame = false;
unsigned long lastSeenMs = 0;
unsigned long lastSampleUs = 0;


enum TrackPhase { LEAVING_START, OUTBOUND, TURNING, RETURNING, STOPPED };
TrackPhase phase = LEAVING_START;
int blackSensors = 0;
int barFrames = 0;
unsigned long turnStarted = 0;
bool turnSawGap = false;
const int turnPWM = 55;
const unsigned long minTurnMs = 300;
const unsigned long turnTimeoutMs = 2000;



void writeMotor(int pwmPin, int dirPin, int speed) {
  speed = constrain(speed, -maxPWM, maxPWM);
  if (speed == 0) {
    analogWrite(pwmPin, 0);
    return;
  }

  const int direction = speed < 0 ? HIGH : LOW;
  if (digitalRead(dirPin) != direction) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, direction);
  }
  analogWrite(pwmPin, abs(speed));
}

bool readLine(float &error, float &peak) {
  ECE3_read_IR(rawSensors);
  float total = 0.0f;
  float weighted = 0.0f;
  peak = 0.0f;
  blackSensors = 0;

  for (int i = 0; i < 8; ++i) {
    const float strength =
      constrain((rawSensors[i] - sensorMin[i]) / sensorSpan[i], 0.0f, 1.0f);
    if (strength > peak) {
      peak = strength;
    }
    if(strength>=0.55f){
      blackSensors++;
    }

    if (strength > noiseFloor) {
      const float weight = strength - noiseFloor;
      total += weight;
      weighted += weight * sensorPosition[i];
    }
  }

  if (peak < lineThreshold) {
    return false;
  }

  error = weighted / total - centerOffset;
  return true;
}

void setup() {
  ECE3_Init();
  if (diagnostics) {
    Serial.begin(115200);
  }

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

void loop() {
  float error = 0.0f;
  float peak = 0.0f;
  const bool lineVisible = readLine(error, peak);
  const unsigned long nowUs = micros();
  const unsigned long nowMs = millis();
  int leftSpeed = 0;
  int rightSpeed = 0;

  const bool onBar = blackSensors >= 7;
  const bool centeredLine = lineVisible && blackSensors <= 3 && fabsf(error) <= 8.0f;

  if (phase == LEAVING_START) {
    if (onBar) {
      // Drive straight off the starting bar.
      writeMotor(leftPWM, leftDir, cornerPWM);
      writeMotor(rightPWM, rightDir, cornerPWM);
      return;
    }
    if (centeredLine) {
      phase = OUTBOUND;
    }
  }

  if (phase == OUTBOUND || phase == RETURNING) {
    barFrames = onBar ? barFrames + 1 : 0;

    if (barFrames >= 2) {
      barFrames = 0;
      if (phase == OUTBOUND) {
        phase = TURNING;
        turnStarted = nowMs;
        turnSawGap = false;
      } else {
        phase = STOPPED;
      }
    }
  }

  if (phase == TURNING) {
    const unsigned long elapsed = nowMs - turnStarted;

    if (!lineVisible) {
      turnSawGap = true;
    }

    if (elapsed >= turnTimeoutMs) {
      phase = STOPPED;
    } else if (elapsed >= minTurnMs && turnSawGap && centeredLine) {
      phase = RETURNING;
      trackedLastFrame = false;
      lastLineSide = 0;
    } else {
      // One wheel forward, one backward.
      writeMotor(leftPWM, leftDir, turnPWM);
      writeMotor(rightPWM, rightDir, -turnPWM);
      return;
    }
  }

  if (phase == STOPPED) {
    writeMotor(leftPWM, leftDir, 0);
    writeMotor(rightPWM, rightDir, 0);
    return;
  }

  if (lineVisible) {
    if (fabsf(error) > 3.0f) {
      lastLineSide = error > 0.0f ? 1 : -1;
    }

    const float dt = (nowUs - lastSampleUs) * 1.0e-6f;
    if (trackedLastFrame && dt > 0.0f) {
      const float rate = (error - lastError) / dt;
      const float alpha = dt / (derivativeFilterSeconds + dt);
      filteredRate += alpha * (rate - filteredRate);
    } else {
      filteredRate = 0.0f;
    }
    

    const float turnFraction = constrain(fabsf(error) / 25.0f, 0.0f, 1.0f);
    const float base = baseSpeed - (baseSpeed - cornerSpeed) * turnFraction;
    const float damping = constrain(kD * filteredRate, -18.0f, 18.0f);
    const float turn = steeringSign * (kP * error + damping);

    leftSpeed = (int)(base + turn);
    rightSpeed = (int)(base - turn);

    lastError = error;
    lastSampleUs = nowUs;
    lastSeenMs = nowMs;
  } else if (lastLineSide != 0 && nowMs - lastSeenMs < searchLimitMs) {
    // Pivot in the direction the last visible line called for.
    leftSpeed = steeringSign * lastLineSide * searchSpeed;
    rightSpeed = -leftSpeed;
  }

  trackedLastFrame = lineVisible;
  writeMotor(leftPWM, leftDir, leftSpeed);
  writeMotor(rightPWM, rightDir, rightSpeed);

  if (diagnostics) {
    static unsigned long lastPrint = 0;
    if (nowMs - lastPrint >= 100) {
      lastPrint = nowMs;
    }
  }
}
