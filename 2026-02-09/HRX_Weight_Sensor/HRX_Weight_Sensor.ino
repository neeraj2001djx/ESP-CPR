#include "HX711.h"

#define HX711_DOUT 19
#define HX711_SCK  18

HX711 scale;

// calibration factor
float calibration_factor = 4200.0;

// zero offset
long zero_offset = 0;

// peak tracking
float peakValue = 0.0;
float lastValue = 0.0;

#define FALL_CONFIRM 2
#define NOISE_THRESH 1.0
int fallCount = 0;

// -------- IDLE STABILITY RECAL --------
#define IDLE_VARIATION    2.0
#define IDLE_TIME_MS 60000UL

float idleMin = 1e9;
float idleMax = -1e9;
unsigned long idleStartTime = 0;
bool idleActive = false;

// =================================================
// Function: Recalibrate zero offset
// =================================================
void recalibrateZero() {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    while (!scale.is_ready());
    sum += scale.read();
  }
  zero_offset = sum / 20;
  Serial.println("### AUTO RECALIBRATED (STABLE IDLE) ###");
}

// =================================================
// Function: Read and process HX711 value
// =================================================
float readValue() {
  long raw = scale.read();

  float value = (raw - zero_offset) / calibration_factor;

  // compression positive
  value = -value;
  if (value < 0) value = 0;

  return value;
}

// =================================================
// Function: Peak detection logic
// =================================================
void handlePeakDetection(float value) {
  if (value > peakValue) {
    peakValue = value;
    fallCount = 0;
  }
  else if (value < lastValue - NOISE_THRESH) {
    fallCount++;
    if (fallCount >= FALL_CONFIRM && peakValue > 0) {
      Serial.print(">>> MAX VALUE: ");
      Serial.println(peakValue, 4);

      peakValue = 0;
      fallCount = 0;
    }
  }

  lastValue = value;
}

// =================================================
// Function: Idle stability & auto recalibration
// =================================================
void handleIdleRecalibration(float value) {
  idleMin = min(idleMin, value);
  idleMax = max(idleMax, value);

  if ((idleMax - idleMin) <= IDLE_VARIATION) {
    if (!idleActive) {
      idleActive = true;
      idleStartTime = millis();
    }
    else if (millis() - idleStartTime >= IDLE_TIME_MS) {
      recalibrateZero();

      idleActive = false;
      idleMin = 1e9;
      idleMax = -1e9;
    }
  }
  else {
    idleActive = false;
    idleMin = 1e9;
    idleMax = -1e9;
  }
}

// =================================================
// SETUP
// =================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  scale.begin(HX711_DOUT, HX711_SCK);

  // initial zero
  recalibrateZero();

  Serial.println("READY");
}

// =================================================
// LOOP
// =================================================
void loop() {
  if (!scale.is_ready()) return;

  float value = readValue();

  Serial.print("Value: ");
  Serial.println(value, 4);

  handlePeakDetection(value);
  handleIdleRecalibration(value);
}
