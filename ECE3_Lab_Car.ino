#include <ECE3.h>
#include <math.h>

// ======================= Pins (unchanged) =======================
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

// ======================= Speed / steering tuning =======================
const int steeringSign = -1;

// Top speed on straights. The return trip has its own cap so you can slow
// down just the complicated half of the track without touching the rest.
const int outboundStraightSpeed = 90;   // was 50
const int returnStraightSpeed   = 80;
const int cornerSpeed           = 45;   // was 32 (32 is near motor stall)
const int maxTrackingPWM        = 170;  // headroom so the outer wheel can push hard

// |error| at which the base speed has fully dropped to cornerSpeed.
const float speedDropError = 20.0f;
// How fast the base speed may climb back up after a corner (PWM per second).
// Braking is instant; accelerating is gradual so it doesn't overshoot S-bends.
const float accelPWMPerSec = 350.0f;

const float kP = 2.6f;                  // was 2.0
const float kD = 0.07f;                 // was 0.04
const float maxDamping = 40.0f;         // was 18
const float derivativeFilterSeconds = 0.025f;

// ======================= U-turn (unchanged logic) =======================
const int pivotPWM = 120;
const int pivotSearchPWM = 90;
const unsigned long pivotSearchStartMs = 380;
const unsigned long pivotTimeoutMs = 1050;
const int guideLockFrames = 2;
const unsigned long barClearMs = 65;
const int clearSpeed = 60;

// ======================= Calibration (unchanged) =======================
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

// ======================= Bar detection =======================
// The slow digital bar read (~6 ms) used to run every loop, which capped the
// control loop at ~100 Hz. Now it only runs when the normal IR read already
// looks like a bar (lots of dark sensors), so the loop runs ~3x faster.
// Set useFastBarGate = false to go back to checking every loop.
const bool useFastBarGate = true;
const float barStrength = 0.50f;
const int barCandidateSensors = 6;

// ======================= Line-loss recovery =======================
const int searchSpeed = 75;                    // was 45 (too weak to spin reliably)
const unsigned long searchLimitMs = 900;       // was 700
const float gapCenterError = 8.0f;             // line lost while centered = gap, not a turn
const unsigned long gapCoastMs = 80;           // drive straight this long before spinning

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
float strength[8];
int strongCount = 0;

float lastError = 0.0f;
float filteredRate = 0.0f;
int lastLineSide = 0;
bool trackedLastFrame = false;
unsigned long lastSeenMs = 0;
unsigned long lastSampleUs = 0;
unsigned long lastFollowUs = 0;
float currentBase = cornerSpeed;

int leftDirState = LOW;
int rightDirState = LOW;

// ======================= Motors =======================
void writeMotor(int pwmPin, int dirPin, int &dirState, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    analogWrite(pwmPin, 0);
    return;
  }

  const int direction = speed < 0 ? HIGH : LOW;
  if (dirState != direction) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, direction);
    dirState = direction;
  }
  analogWrite(pwmPin, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  writeMotor(leftPWM, leftDir, leftDirState, leftSpeed);
  writeMotor(rightPWM, rightDir, rightDirState, rightSpeed);
}

// ======================= Sensors =======================
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

// One IR read per loop; everything else uses these normalized values.
void readSensors() {
  ECE3_read_IR(rawSensors);
  strongCount = 0;
  for (int i = 0; i < 8; ++i) {
    const float s =
      constrain((rawSensors[i] - sensorMin[i]) / sensorSpan[i], 0.0f, 1.0f);
    strength[i] = s;
    if (s >= barStrength) {
      ++strongCount;
    }
  }
}

bool checkBar() {
  if (useFastBarGate && strongCount < barCandidateSensors) {
    return false;
  }
  return readBarSensorCount() >= minBarSensors;
}

// Finds the line using only the peak sensor and its two neighbours.
// If more than one dark patch is visible (crossings, hairpins where both
// arms are under the bar, stray marks), it follows the patch closest to
// where the line was last frame instead of averaging them together.
bool findLine(float &error, float &centerStrength) {
  centerStrength = strength[3] > strength[4] ? strength[3] : strength[4];

  int best = -1;
  float bestScore = 1.0e9f;
  for (int i = 0; i < 8; ++i) {
    const float s = strength[i];
    if (s < lineThreshold) {
      continue;
    }
    const float left = i > 0 ? strength[i - 1] : 0.0f;
    const float right = i < 7 ? strength[i + 1] : 0.0f;
    if (s < left || s < right) {
      continue;  // not a local peak
    }
    const float score = trackedLastFrame
      ? fabsf(sensorPosition[i] - centerOffset - lastError)
      : -s;
    if (score < bestScore) {
      bestScore = score;
      best = i;
    }
  }

  if (best < 0) {
    return false;
  }

  float total = 0.0f;
  float weighted = 0.0f;
  for (int i = best - 1; i <= best + 1; ++i) {
    if (i < 0 || i > 7) {
      continue;
    }
    if (strength[i] > noiseFloor) {
      const float w = strength[i] - noiseFloor;
      total += w;
      weighted += w * sensorPosition[i];
    }
  }
  if (total <= 0.0f) {
    return false;
  }
  error = weighted / total - centerOffset;
  return true;
}

// ======================= Line following =======================
void resetTracking() {
  trackedLastFrame = false;
  filteredRate = 0.0f;
  currentBase = cornerSpeed;
}

void followGuide(bool lineVisible, float error) {
  const unsigned long nowUs = micros();
  const unsigned long nowMs = millis();
  float loopDt = (nowUs - lastFollowUs) * 1.0e-6f;
  if (loopDt > 0.05f) {
    loopDt = 0.05f;
  }
  lastFollowUs = nowUs;

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

    // Speed: fast on straights, drop hard as soon as the error grows,
    // ramp back up gradually.
    const float topSpeed =
      (trackState == OUTBOUND) ? outboundStraightSpeed : returnStraightSpeed;
    const float turnFraction =
      constrain((fabsf(error) - 3.0f) / (speedDropError - 3.0f), 0.0f, 1.0f);
    const float targetBase = topSpeed - (topSpeed - cornerSpeed) * turnFraction;
    if (targetBase < currentBase) {
      currentBase = targetBase;
    } else {
      currentBase += accelPWMPerSec * loopDt;
      if (currentBase > targetBase) {
        currentBase = targetBase;
      }
    }

    const float damping = constrain(kD * filteredRate, -maxDamping, maxDamping);
    const float turn = steeringSign * (kP * error + damping);

    leftSpeed = constrain((int)(currentBase + turn), -maxTrackingPWM, maxTrackingPWM);
    rightSpeed = constrain((int)(currentBase - turn), -maxTrackingPWM, maxTrackingPWM);

    lastError = error;
    lastSampleUs = nowUs;
    lastSeenMs = nowMs;
  } else {
    const unsigned long lostMs = nowMs - lastSeenMs;
    currentBase = cornerSpeed;

    if (fabsf(lastError) < gapCenterError && lostMs < gapCoastMs) {
      // Line vanished while centered: probably a gap, keep going straight.
      leftSpeed = cornerSpeed;
      rightSpeed = cornerSpeed;
    } else if (lastLineSide != 0 && lostMs < searchLimitMs) {
      // Line ran off one edge (sharp turn): spin hard toward that side.
      leftSpeed = steeringSign * lastLineSide * searchSpeed;
      rightSpeed = -leftSpeed;
    }
  }

  trackedLastFrame = lineVisible;
  drive(leftSpeed, rightSpeed);
}

// ======================= Setup / loop =======================
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

  digitalWrite(leftDir, LOW);
  digitalWrite(rightDir, LOW);
  leftDirState = LOW;
  rightDirState = LOW;
  drive(0, 0);
  digitalWrite(leftSleep, HIGH);
  digitalWrite(rightSleep, HIGH);
  delay(1000);
  lastFollowUs = micros();
}

void loop() {
  if (trackState == STOPPED) {
    return;
  }

  const unsigned long nowMs = millis();
  readSensors();
  const bool onBar = checkBar();

  if (diagnostics) {
    static unsigned long lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 100) {
      lastPrintMs = nowMs;
      Serial.print("state ");
      Serial.print((int)trackState);
      Serial.print("  err ");
      Serial.print(lastError);
      Serial.print("  dark ");
      Serial.print(strongCount);
      Serial.print("  base ");
      Serial.println((int)currentBase);
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
    trackedLastFrame = false;  // pick the strongest patch while spinning
    const bool lineVisible = findLine(error, centerStrength);
    const bool centeredGuide = !onBar && lineVisible &&
      centerStrength >= pivotCenterStrength && fabsf(error) <= pivotCenterError;
    centeredFrames = centeredGuide ? centeredFrames + 1 : 0;

    if (centeredFrames >= guideLockFrames) {
      resetTracking();
      lastLineSide = 0;
      lastError = error;
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
    drive(clearSpeed, clearSpeed);
    return;
  }

  const bool lineVisible = findLine(error, centerStrength);
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
