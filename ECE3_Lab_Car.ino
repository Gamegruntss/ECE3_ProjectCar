#include <ECE3.h>
#include <math.h>

// ======================= Pins =======================
const int leftSleep = 31;
const int leftDir = 29;
const int leftPWM = 40;
const int rightSleep = 11;
const int rightDir = 30;
const int rightPWM = 39;
const int evenLed = 45;
const int oddLed = 61;

// ======================= Speed =======================
const int steeringSign = -1;
const int straightSpeed = 150;      // centred on a straight
const int cornerSpeed   = 40;       // base speed in the tightest turns
const float slowDownError = 18.0f;  // line this far off-centre -> cornerSpeed
const float slowDownRate  = 300.0f; // line moving this fast    -> cornerSpeed (brakes before the error gets big)
const float accelPerSec   = 600.0f; // speed climbs back gradually, brakes instantly
const int maxPWM = 235;

// ======================= Steering =======================
// turn = kP*e + kP2*e*|e| + kD*rate
// Soft near the centre (no weaving), very strong at the edges. When turn is
// bigger than the base speed the inner wheel runs in reverse.
//   error 5 -> ~11   error 20 -> ~64   error 35 -> ~143
const float kP  = 2.0f;
const float kP2 = 0.06f;
const float kD  = 0.15f;
const float derivativeFilterSeconds = 0.02f;

// ======================= U-turn / stop (unchanged) =======================
const int pivotPWM = 120;
const int pivotSearchPWM = 90;
const unsigned long pivotSearchStartMs = 380;
const unsigned long pivotTimeoutMs = 1050;
const int guideLockFrames = 2;
const unsigned long barClearMs = 65;
const int clearSpeed = 70;

// Bar = all 8 sensors black for 3 readings AND the car was centred on a
// normal line just before (a corner always pulls the line to one side first).
const float barStrength = 0.60f;
const int barConfirmFrames = 3;
const float barCenteredError = 10.0f;
const unsigned long barCenteredWithinMs = 60;

// ======================= Calibration =======================
const float sensorMin[8] = {
  749.75f, 646.25f, 669.25f, 549.75f, 645.75f, 634.50f, 697.25f, 782.50f
};
const float sensorSpan[8] = {
  1750.25f, 1840.00f, 1830.75f, 947.50f, 1450.50f, 1851.50f, 1802.75f, 1717.50f
};
const float sensorPosition[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
const float centerOffset = 1.04f;
const float noiseFloor = 0.12f;
const float lineThreshold = 0.30f;
const float pivotCenterStrength = 0.45f;
const float pivotCenterError = 8.0f;

const bool diagnostics = false;     // serial printing off = faster loop

// ======================= State =======================
enum TrackState { OUTBOUND, PIVOTING, CLEARING_FAR_BAR, RETURNING, STOPPED };
TrackState trackState = OUTBOUND;

unsigned long pivotStartedMs = 0;
int centeredFrames = 0;
bool clearingOnGuide = false;
unsigned long clearStartedMs = 0;

uint16_t rawSensors[8];
int darkCount = 0;
int barFrames = 0;
unsigned long lastCenteredMs = 0;

float lastError = 0.0f;
float filteredRate = 0.0f;
bool trackedLastFrame = false;
float currentBase = cornerSpeed;
unsigned long lastSampleUs = 0;
unsigned long lastFollowUs = 0;

int leftDirState = LOW;
int rightDirState = LOW;

// ======================= Motors =======================
void writeMotor(int pwmPin, int dirPin, int &dirState, int speed) {
  speed = constrain(speed, -255, 255);
  const int direction = speed < 0 ? HIGH : LOW;
  if (speed != 0 && dirState != direction) {
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
bool readLine(float &error, float &centerStrength) {
  ECE3_read_IR(rawSensors);
  float total = 0.0f;
  float weighted = 0.0f;
  float peak = 0.0f;
  float s3 = 0.0f;
  float s4 = 0.0f;
  darkCount = 0;

  for (int i = 0; i < 8; ++i) {
    const float s =
      constrain((rawSensors[i] - sensorMin[i]) / sensorSpan[i], 0.0f, 1.0f);
    if (i == 3) s3 = s;
    if (i == 4) s4 = s;
    if (s > peak) peak = s;
    if (s >= barStrength) ++darkCount;
    if (s > noiseFloor) {
      total += s - noiseFloor;
      weighted += (s - noiseFloor) * sensorPosition[i];
    }
  }
  centerStrength = s3 > s4 ? s3 : s4;

  if (peak < lineThreshold || total <= 0.0f) return false;
  error = weighted / total - centerOffset;
  return true;
}

bool barDetected(unsigned long nowMs, bool lineVisible, float error) {
  if (lineVisible && darkCount <= 4 && fabsf(error) <= barCenteredError) {
    lastCenteredMs = nowMs;
  }
  if (darkCount == 8) {
    if (barFrames == 0 && nowMs - lastCenteredMs > barCenteredWithinMs) {
      barFrames = -1000;   // went all-black from off-centre = a corner, ignore
    }
    ++barFrames;
  } else {
    barFrames = 0;
  }
  return barFrames >= barConfirmFrames;
}

// ======================= Line following =======================
// One continuous PD loop. If the line slips off the sensors for a moment,
// the motors simply keep their last command (already turning hard toward
// the line), so the turn carries straight on - no search, no spinning back.
void followLine(bool lineVisible, float error) {
  const unsigned long nowUs = micros();
  float loopDt = (nowUs - lastFollowUs) * 1.0e-6f;
  if (loopDt > 0.05f) loopDt = 0.05f;
  lastFollowUs = nowUs;

  if (!lineVisible) {
    trackedLastFrame = false;
    return;   // hold last motor command
  }

  const float dt = (nowUs - lastSampleUs) * 1.0e-6f;
  if (trackedLastFrame && dt > 0.0f && dt < 0.05f) {
    const float rate = (error - lastError) / dt;
    filteredRate += (dt / (derivativeFilterSeconds + dt)) * (rate - filteredRate);
  } else {
    filteredRate = 0.0f;
  }

  // Slow down when the line is off-centre OR moving sideways fast.
  const float slowE = fabsf(error) / slowDownError;
  const float slowR = fabsf(filteredRate) / slowDownRate;
  const float slow = constrain(slowE > slowR ? slowE : slowR, 0.0f, 1.0f);
  const float targetBase = straightSpeed - (straightSpeed - cornerSpeed) * slow;
  if (targetBase < currentBase) {
    currentBase = targetBase;
  } else {
    currentBase += accelPerSec * loopDt;
    if (currentBase > targetBase) currentBase = targetBase;
  }

  const float turn = steeringSign *
    (kP * error + kP2 * error * fabsf(error) + kD * filteredRate);

  drive(constrain((int)(currentBase + turn), -maxPWM, maxPWM),
        constrain((int)(currentBase - turn), -maxPWM, maxPWM));

  lastError = error;
  lastSampleUs = nowUs;
  trackedLastFrame = true;
}

// ======================= Setup / loop =======================
void setup() {
  ECE3_Init();
  if (diagnostics) Serial.begin(115200);

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
  drive(0, 0);
  digitalWrite(leftSleep, HIGH);
  digitalWrite(rightSleep, HIGH);
  delay(1000);
  lastFollowUs = micros();
}

void loop() {
  if (trackState == STOPPED) return;

  const unsigned long nowMs = millis();
  float error = 0.0f;
  float centerStrength = 0.0f;
  const bool lineVisible = readLine(error, centerStrength);
  const bool onBar = barDetected(nowMs, lineVisible, error);
  const bool wideDark = darkCount >= 6;

  if (trackState == OUTBOUND && onBar) {
    pivotStartedMs = nowMs;
    centeredFrames = 0;
    trackState = PIVOTING;
    drive(-pivotPWM, pivotPWM);
    if (diagnostics) Serial.println("Far bar. Pivoting.");
    return;
  }

  if (trackState == RETURNING && onBar) {
    drive(0, 0);
    trackState = STOPPED;
    if (diagnostics) Serial.println("Start bar. Stopped.");
    return;
  }

  if (trackState == PIVOTING) {
    const unsigned long elapsed = nowMs - pivotStartedMs;
    if (elapsed < pivotSearchStartMs) return;
    if (elapsed >= pivotTimeoutMs) {
      drive(0, 0);
      trackState = STOPPED;
      if (diagnostics) Serial.println("Pivot timed out.");
      return;
    }
    drive(-pivotSearchPWM, pivotSearchPWM);
    const bool centered = !wideDark && lineVisible &&
      centerStrength >= pivotCenterStrength && fabsf(error) <= pivotCenterError;
    centeredFrames = centered ? centeredFrames + 1 : 0;
    if (centeredFrames >= guideLockFrames) {
      trackedLastFrame = false;
      currentBase = cornerSpeed;
      clearingOnGuide = false;
      trackState = CLEARING_FAR_BAR;
      if (diagnostics) Serial.println("Guide found. Driving back.");
    }
    return;
  }

  if (trackState == CLEARING_FAR_BAR) {
    if (wideDark || !lineVisible) {
      clearingOnGuide = false;
      trackedLastFrame = false;
      drive(clearSpeed, clearSpeed);
      return;
    }
    if (!clearingOnGuide) {
      clearStartedMs = nowMs;
      clearingOnGuide = true;
    } else if (nowMs - clearStartedMs >= barClearMs) {
      barFrames = 0;
      trackState = RETURNING;
    }
  }

  followLine(lineVisible, error);
}
