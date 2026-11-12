#include <ECE3.h>
#include <math.h>

// ======================= Pins (unchanged) =======================
const int leftSleep = 31;
const int leftDir = 29;
const int leftPWM = 40;
const int rightSleep = 11;
const int rightDir = 30;
const int rightPWM = 39;

const int evenLed = 45;
const int oddLed = 61;

// ======================= Speed / steering tuning =======================
const int steeringSign = -1;

const int outboundStraightSpeed = 115;  // v2: 90
const int returnStraightSpeed   = 105;  // v2: 80
const int cornerSpeed           = 55;   // v2: 45
const int maxTrackingPWM        = 210;

// |error| at which the base speed has fully dropped to cornerSpeed.
const float speedDropError = 18.0f;
// Braking is instant; accelerating back up is limited to this (PWM per second).
const float accelPWMPerSec = 550.0f;

const float kP = 3.0f;
const float kD = 0.09f;
const float maxDamping = 50.0f;
const float derivativeFilterSeconds = 0.02f;

// ======================= U-turn (unchanged logic) =======================
const int pivotPWM = 120;
const int pivotSearchPWM = 90;
const unsigned long pivotSearchStartMs = 380;
const unsigned long pivotTimeoutMs = 1050;
const int guideLockFrames = 2;
const unsigned long barClearMs = 65;
const int clearSpeed = 70;

// ======================= Calibration (unchanged) =======================
const float sensorMin[8] = {
  749.75f, 646.25f, 669.25f, 549.75f, 645.75f, 634.50f, 697.25f, 782.50f
};
const float sensorSpan[8] = {
  1750.25f, 1840.00f, 1830.75f, 947.50f, 1450.50f, 1851.50f, 1802.75f, 1717.50f
};
const float sensorPosition[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
const float centerOffset = 1.04f;
const float noiseFloor = 0.15f;      // v2: 0.10
const float lineThreshold = 0.35f;   // v2: 0.20 - stricter "this is the line"
const float pivotCenterStrength = 0.45f;
const float pivotCenterError = 8.0f;

// ======================= Bar detection (strict) =======================
// A real bar is only accepted if ALL of these are true:
//  1. All 8 sensors read solidly black (calibrated strength >= barStrength).
//  2. Just before it, the car was driving centred on a normal-width line
//     (|error| <= barApproachError) for at least barApproachMs.
//  3. It went from a normal line to all-black quickly (<= barOnsetMs).
//     A sharp turn fills the sensors gradually from one side while the
//     car is already steering, so it fails 2 and/or 3.
//  4. All-black lasts for barConfirmFrames readings in a row.
const float barStrength = 0.60f;
const float barApproachError = 9.0f;
const unsigned long barApproachMs = 80;
const unsigned long barOnsetMs = 45;
const int barConfirmFrames = 3;
const int narrowLineMax = 3;      // a normal line covers at most this many sensors
const int wideDarkSensors = 6;    // "mostly black" - used only while crossing the far bar

// ======================= Line-loss recovery =======================
const int searchSpeed = 80;
const unsigned long searchLimitMs = 900;
const float gapCenterError = 8.0f;
const unsigned long gapCoastMs = 70;

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
int darkCount = 0;    // sensors >= barStrength
int lineCount = 0;    // sensors >= lineThreshold

// Bar detector state
unsigned long lastNarrowMs = 0;
unsigned long centeredSinceMs = 0;
bool centeredRun = false;
bool approachOk = false;
bool onsetOk = false;
int fullDarkFrames = 0;

// Line follower state
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
void readSensors() {
  ECE3_read_IR(rawSensors);
  darkCount = 0;
  lineCount = 0;
  for (int i = 0; i < 8; ++i) {
    const float s =
      constrain((rawSensors[i] - sensorMin[i]) / sensorSpan[i], 0.0f, 1.0f);
    strength[i] = s;
    if (s >= barStrength) ++darkCount;
    if (s >= lineThreshold) ++lineCount;
  }
}

// Follows the dark patch closest to where the line was last reading, using
// only the peak sensor and its two neighbours. Side branches, crossings and
// the other arm of a hairpin are ignored instead of averaged in.
bool findLine(float &error, float &centerStrength) {
  centerStrength = strength[3] > strength[4] ? strength[3] : strength[4];

  int best = -1;
  float bestScore = 1.0e9f;
  for (int i = 0; i < 8; ++i) {
    const float s = strength[i];
    if (s < lineThreshold) continue;
    const float left = i > 0 ? strength[i - 1] : 0.0f;
    const float right = i < 7 ? strength[i + 1] : 0.0f;
    if (s < left || s < right) continue;  // not a local peak
    const float score = trackedLastFrame
      ? fabsf(sensorPosition[i] - centerOffset - lastError)
      : -s;
    if (score < bestScore) {
      bestScore = score;
      best = i;
    }
  }
  if (best < 0) return false;

  float total = 0.0f;
  float weighted = 0.0f;
  for (int i = best - 1; i <= best + 1; ++i) {
    if (i < 0 || i > 7) continue;
    if (strength[i] > noiseFloor) {
      const float w = strength[i] - noiseFloor;
      total += w;
      weighted += w * sensorPosition[i];
    }
  }
  if (total <= 0.0f) return false;
  error = weighted / total - centerOffset;
  return true;
}

// Returns true only for a real bar (see rules at the top).
bool updateBarDetector(unsigned long nowMs, bool lineVisible, float error) {
  if (lineCount <= narrowLineMax) {
    lastNarrowMs = nowMs;
    if (lineVisible && fabsf(error) <= barApproachError) {
      if (!centeredRun) {
        centeredRun = true;
        centeredSinceMs = nowMs;
      }
    } else {
      centeredRun = false;
    }
    approachOk = centeredRun && (nowMs - centeredSinceMs >= barApproachMs);
  }

  if (darkCount == 8) {
    if (fullDarkFrames == 0) {
      onsetOk = approachOk && (nowMs - lastNarrowMs <= barOnsetMs);
      if (diagnostics && !onsetOk &&
          (trackState == OUTBOUND || trackState == RETURNING)) {
        Serial.print("Ignored all-black (");
        Serial.print(approachOk ? "slow onset" : "not centred before");
        Serial.println(") - treating as a turn");
      }
    }
    ++fullDarkFrames;
  } else {
    fullDarkFrames = 0;
    onsetOk = false;
  }

  return onsetOk && fullDarkFrames >= barConfirmFrames;
}

void resetBarDetector() {
  centeredRun = false;
  approachOk = false;
  onsetOk = false;
  fullDarkFrames = 0;
}

// ======================= Line following =======================
void followGuide(bool lineVisible, float error) {
  const unsigned long nowUs = micros();
  const unsigned long nowMs = millis();
  float loopDt = (nowUs - lastFollowUs) * 1.0e-6f;
  if (loopDt > 0.05f) loopDt = 0.05f;
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

    const float topSpeed =
      (trackState == OUTBOUND) ? outboundStraightSpeed : returnStraightSpeed;
    const float turnFraction =
      constrain((fabsf(error) - 3.0f) / (speedDropError - 3.0f), 0.0f, 1.0f);
    const float targetBase = topSpeed - (topSpeed - cornerSpeed) * turnFraction;
    if (targetBase < currentBase) {
      currentBase = targetBase;
    } else {
      currentBase += accelPWMPerSec * loopDt;
      if (currentBase > targetBase) currentBase = targetBase;
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
      leftSpeed = cornerSpeed;   // centred when lost: probably a gap
      rightSpeed = cornerSpeed;
    } else if (lastLineSide != 0 && lostMs < searchLimitMs) {
      leftSpeed = steeringSign * lastLineSide * searchSpeed;  // ran off an edge
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

  float error = 0.0f;
  float centerStrength = 0.0f;
  const bool lineVisible = findLine(error, centerStrength);
  const bool barConfirmed = updateBarDetector(nowMs, lineVisible, error);
  const bool wideDark = darkCount >= wideDarkSensors;

  if (diagnostics) {
    static unsigned long lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 100) {
      lastPrintMs = nowMs;
      Serial.print("state ");
      Serial.print((int)trackState);
      Serial.print("  err ");
      Serial.print(lineVisible ? error : 99.0f);
      Serial.print("  dark ");
      Serial.print(darkCount);
      Serial.print("  base ");
      Serial.println((int)currentBase);
    }
  }

  if (trackState == OUTBOUND && barConfirmed) {
    pivotStartedMs = millis();
    centeredFrames = 0;
    trackedLastFrame = false;
    trackState = PIVOTING;
    drive(-pivotPWM, pivotPWM);
    if (diagnostics) {
      Serial.println("Far bar confirmed. Pivoting.");
    }
    return;
  }

  if (trackState == RETURNING && barConfirmed) {
    drive(0, 0);
    trackState = STOPPED;
    if (diagnostics) {
      Serial.println("Start bar confirmed. Stopped.");
    }
    return;
  }

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
    const bool centeredGuide = !wideDark && lineVisible &&
      centerStrength >= pivotCenterStrength && fabsf(error) <= pivotCenterError;
    centeredFrames = centeredGuide ? centeredFrames + 1 : 0;

    if (centeredFrames >= guideLockFrames) {
      trackedLastFrame = false;
      filteredRate = 0.0f;
      currentBase = cornerSpeed;
      lastLineSide = 0;
      lastError = error;
      clearingOnGuide = false;
      resetBarDetector();
      trackState = CLEARING_FAR_BAR;
      followGuide(true, error);
      if (diagnostics) {
        Serial.println("Guide line found. Driving back.");
      }
    }
    return;
  }

  if (trackState == CLEARING_FAR_BAR) {
    if (wideDark) {
      clearingOnGuide = false;
      trackedLastFrame = false;
      drive(clearSpeed, clearSpeed);
      return;
    }
    if (!lineVisible) {
      clearingOnGuide = false;
    } else if (!clearingOnGuide) {
      clearStartedMs = nowMs;
      clearingOnGuide = true;
    } else if (nowMs - clearStartedMs >= barClearMs) {
      resetBarDetector();
      trackState = RETURNING;
    }
  }

  followGuide(lineVisible, error);
}
