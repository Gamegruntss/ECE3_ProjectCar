#include <ECE3.h>
#include <math.h>

const int leftSleep = 31;
const int leftDir = 29;
const int leftPWM = 40;
const int rightSleep = 11;
const int rightDir = 30;
const int rightPWM = 39;

const int reflectancePins[8] = {65, 48, 64, 47, 52, 68, 53, 69};
const int evenLed = 45;
const int oddLed = 61;
const int barReadDelayUs = 700;
const int minBarSensors = 8;

const int steeringSign = -1;
const int baseSpeed = 50;
const int cornerSpeed = 32;
const int maxTrackingPWM = 100;
const float kP = 2.0f;
const float kD = 0.04f;
const float derivativeFilterSeconds = 0.035f;

const int pivotPWM = 120;
const int pivotSearchPWM = 90;
const unsigned long pivotSearchStartMs = 380;
const unsigned long pivotTimeoutMs = 1050;
const int guideLockFrames = 2;
const unsigned long barClearMs = 65;

const float sensorMin[8] = {
  749.75f, 646.25f, 669.25f, 549.75f, 645.75f, 634.50f, 697.25f, 782.50f
};
const float sensorSpan[8] = {
  1750.25f, 1840.00f, 1830.75f, 947.50f, 1450.50f, 1851.50f, 1802.75f, 1717.50f
};
const float sensorPosition[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
const float centerOffset = 1.04f;
const float noiseFloor = 0.10f;
const float lineThreshold = 0.20f;
const float pivotCenterStrength = 0.45f;
const float pivotCenterError = 8.0f;

const int searchSpeed = 45;
const unsigned long searchLimitMs = 700;
const bool diagnostics = true;

enum TrackState {
  OUTBOUND,
  PIVOTING,
  CLEARING_FAR_BAR,
  RETURNING,
  STOPPED
};

TrackState trackState = OUTBOUND;
unsigned long pivotStartedMs = 0;
int centeredFrames = 0;
bool clearingOnGuide = false;
unsigned long clearStartedMs = 0;

uint16_t rawSensors[8];
float lastError = 0.0f;
float filteredRate = 0.0f;
int lastLineSide = 0;
bool trackedLastFrame = false;
unsigned long lastSeenMs = 0;
unsigned long lastSampleUs = 0;

void writeMotor(int pwmPin, int dirPin, int speed) {
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

void drive(int leftSpeed, int rightSpeed) {
  writeMotor(leftPWM, leftDir, leftSpeed);
  writeMotor(rightPWM, rightDir, rightSpeed);
}

int readBarSensorCount() {
  // Re-enable both emitters before the direct pin reads.
  digitalWrite(oddLed, HIGH);
  digitalWrite(evenLed, HIGH);
  delayMicroseconds(200);

  int highCount = 0;
  for (int i = 0; i < 8; ++i) {
    const int pin = reflectancePins[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    pinMode(pin, INPUT);
    delayMicroseconds(barReadDelayUs);
    if (digitalRead(pin) == HIGH) {
      ++highCount;
    }
  }
  return highCount;
}

bool readLine(float &error, float &centerStrength) {
  ECE3_read_IR(rawSensors);
  float total = 0.0f;
  float weighted = 0.0f;
  float peak = 0.0f;
  centerStrength = 0.0f;

  for (int i = 0; i < 8; ++i) {
    const float strength =
      constrain((rawSensors[i] - sensorMin[i]) / sensorSpan[i], 0.0f, 1.0f);
    if (strength > peak) {
      peak = strength;
    }
    if ((i == 3 || i == 4) && strength > centerStrength) {
      centerStrength = strength;
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

void followGuide(bool lineVisible, float error) {
  const unsigned long nowUs = micros();
  const unsigned long nowMs = millis();
  int leftSpeed = 0;
  int rightSpeed = 0;

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

    leftSpeed = constrain((int)(base + turn), -maxTrackingPWM, maxTrackingPWM);
    rightSpeed = constrain((int)(base - turn), -maxTrackingPWM, maxTrackingPWM);

    lastError = error;
    lastSampleUs = nowUs;
    lastSeenMs = nowMs;
  } else if (lastLineSide != 0 && nowMs - lastSeenMs < searchLimitMs) {
    leftSpeed = steeringSign * lastLineSide * searchSpeed;
    rightSpeed = -leftSpeed;
  }

  trackedLastFrame = lineVisible;
  drive(leftSpeed, rightSpeed);
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
  pinMode(oddLed, OUTPUT);
  pinMode(evenLed, OUTPUT);

  drive(0, 0);
  digitalWrite(leftDir, LOW);
  digitalWrite(rightDir, LOW);
  digitalWrite(leftSleep, HIGH);
  digitalWrite(rightSleep, HIGH);
  delay(1000);
}

void loop() {
  if (trackState == STOPPED) {
    return;
  }

  const int highCount = readBarSensorCount();
  const bool onBar = highCount >= minBarSensors;
  const unsigned long nowMs = millis();

  if (diagnostics) {
    static unsigned long lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 100) {
      lastPrintMs = nowMs;
      Serial.print("Black sensors: ");
      Serial.println(highCount);
    }
  }

  if (trackState == OUTBOUND && onBar) {
    pivotStartedMs = millis();
    centeredFrames = 0;
    trackState = PIVOTING;
    drive(-pivotPWM, pivotPWM);
    if (diagnostics) {
      Serial.println("Far bar detected. Pivoting.");
    }
    return;
  }

  if (trackState == RETURNING && onBar) {
    drive(0, 0);
    trackState = STOPPED;
    if (diagnostics) {
      Serial.println("Start bar detected. Stopped.");
    }
    return;
  }

  float error = 0.0f;
  float centerStrength = 0.0f;

  if (trackState == PIVOTING) {
    const unsigned long pivotElapsed = nowMs - pivotStartedMs;
    if (pivotElapsed < pivotSearchStartMs) {
      return;
    }

    if (pivotElapsed >= pivotTimeoutMs) {
      drive(0, 0);
      trackState = STOPPED;
      if (diagnostics) {
        Serial.println("Pivot timed out. Guide line not found.");
      }
      return;
    }

    drive(-pivotSearchPWM, pivotSearchPWM);
    const bool lineVisible = readLine(error, centerStrength);
    const bool centeredGuide = !onBar && lineVisible &&
      centerStrength >= pivotCenterStrength && fabsf(error) <= pivotCenterError;
    centeredFrames = centeredGuide ? centeredFrames + 1 : 0;

    if (centeredFrames >= guideLockFrames) {
      trackedLastFrame = false;
      filteredRate = 0.0f;
      lastLineSide = 0;
      clearingOnGuide = false;
      trackState = CLEARING_FAR_BAR;
      followGuide(true, error);
      if (diagnostics) {
        Serial.println("Guide line found. Driving back.");
      }
    }
    return;
  }

  if (trackState == CLEARING_FAR_BAR && onBar) {
    clearingOnGuide = false;
    trackedLastFrame = false;
    drive(baseSpeed, baseSpeed);
    return;
  }

  const bool lineVisible = readLine(error, centerStrength);
  if (trackState == CLEARING_FAR_BAR) {
    if (!lineVisible) {
      clearingOnGuide = false;
    } else if (!clearingOnGuide) {
      clearStartedMs = nowMs;
      clearingOnGuide = true;
    } else if (nowMs - clearStartedMs >= barClearMs) {
      trackState = RETURNING;
    }
  }

  followGuide(lineVisible, error);
}
