#include <ECE3.h>


const int leftSleep  = 31;
const int leftDir    = 29;
const int leftPWM    = 40;

const int rightSleep = 11;
const int rightDir   = 30;
const int rightPWM   = 39;


const int   baseSpeed    = 65;   // speed of both wheels when we're centered on the line
const float kP           = 2.0;  // how hard we steer for each unit of error
const int   steeringSign = 1;    // flip to -1 if the car steers AWAY from the line


// The 8-4-2-1 pattern, divided by 4. Left sensors are negative,
// right sensors are positive, and the outer ones count the most.
// Order must match the order the sensors were in during calibration.
const float weights[8] = { -2.0, -1.0, -0.5, -0.25, 0.25, 0.5, 1.0, 2.0 };

// Sensor values (raw readings, then the 5-sample average)
uint16_t rawValues[8];
float    sensorAvg[8];


// During calibration we moved the car across the line and wrote down
// the weighted reading at each position. Each position lines up with
// the reading in the same spot of the other list.
//
// We stop at +28 because the +32 reading "folded back" and came out
// smaller than the +28 one, which would confuse the lookup.
const int tableSize = 16;

const float positions[tableSize] = {
  -32, -28, -24, -20, -16, -12, -8, -4,
    0,   4,   8,  12,  16,  20, 24, 28
};

const float readings[tableSize] = {
  -2226.719199,  -1560.400390,  -1128.548502,  -725.1264949,
   -582.3227173,  -450.0611413,  -304.1680368,  -183.2196190,
     83.91175667,  298.6762051,   401.1685668,   607.4761230,
    727.3684738,  1090.597989,   1388.940946,   2301.324419
};


void setup() {
  ECE3_Init();          // same init as the calibration sketch
  Serial.begin(9600);

  // All motor pins are outputs
  pinMode(leftSleep, OUTPUT);
  pinMode(leftDir, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightSleep, OUTPUT);
  pinMode(rightDir, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // Start stopped, both wheels set to go forward
  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, 0);
  digitalWrite(leftDir, LOW);
  digitalWrite(rightDir, LOW);

  // Wake up the motor drivers
  digitalWrite(leftSleep, HIGH);
  digitalWrite(rightSleep, HIGH);

  delay(1000);          // one second to set the car down
}


// calibration used, so the numbers stay comparable.
void readSensors() {
  long sums[8] = {0};

  for (int sample = 0; sample < 5; sample++) {
    ECE3_read_IR(rawValues);
    for (int i = 0; i < 8; i++) {
      sums[i] += rawValues[i];
    }
  }

  for (int i = 0; i < 8; i++) {
    sensorAvg[i] = sums[i] / 5.0;
  }
}


//  Squash 8 sensors down into one number
// Negative = line is toward the left side, positive = toward the right.
float getWeightedReading() {
  float total = 0;
  for (int i = 0; i < 8; i++) {
    total += weights[i] * sensorAvg[i];
  }
  return total;
}


//  Turn the weighted reading into a position error
// Look the reading up in the calibration table. If it lands between
// two entries, draw a straight line between them (linear interpolation).
float readingToError(float reading) {
  // Past either end of the table? Just use the end value.
  if (reading <= readings[0])             return positions[0];
  if (reading >= readings[tableSize - 1]) return positions[tableSize - 1];

  // Find the first entry bigger than our reading.
  // Our reading sits between entry i-1 and entry i.
  for (int i = 1; i < tableSize; i++) {
    if (reading <= readings[i]) {
      float low  = readings[i - 1];
      float high = readings[i];

      // How far between the two entries we are (0 = at low, 1 = at high)
      float fraction = (reading - low) / (high - low);

      return positions[i - 1] + fraction * (positions[i] - positions[i - 1]);
    }
  }

  return 0;  // shouldn't ever get here
}


//  Print what's going on, every half second

void printDebug(float weighted, float error, int leftSpeed, int rightSpeed) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 500) return;   // not time yet
  lastPrint = millis();

  Serial.print("Sensors: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorAvg[i], 0);
    Serial.print(" ");
  }

  Serial.print("| Weighted: ");
  Serial.print(weighted, 1);
  Serial.print(" | Error: ");
  Serial.print(error, 1);
  Serial.print(" | Motors: ");
  Serial.print(leftSpeed);
  Serial.print(" ");
  Serial.println(rightSpeed);
}


//  Main loop - runs over and over

void loop() {
  // 1. Where is the line?
  readSensors();
  float weighted = getWeightedReading();
  float error    = readingToError(weighted);

  // 2. How hard should we turn? (proportional control)
  float correction = steeringSign * kP * error;

  // Don't let the correction be bigger than the base speed,
  // otherwise one wheel would get a negative speed.
  if (correction >  baseSpeed) correction =  baseSpeed;
  if (correction < -baseSpeed) correction = -baseSpeed;

  // 3. Speed one wheel up and slow the other down
  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  analogWrite(leftPWM, leftSpeed);
  analogWrite(rightPWM, rightSpeed);

  // 4. Show some numbers on the Serial Monitor
  printDebug(weighted, error, leftSpeed, rightSpeed);
}
