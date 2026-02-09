#include "HX711.h"

// =================================================
// HX711 CONFIG
// =================================================
#define HX711_DOUT 19
#define HX711_SCK  18

HX711 scale;

float calibration_factor = 4200.0;
long zero_offset = 0;

// HX peak detection
float peakValue = 0.0;
float lastValue = 0.0;

#define FALL_CONFIRM 2
#define NOISE_THRESH 1.0
int fallCount = 0;

// HX idle recalibration
#define IDLE_VARIATION  2.0
#define IDLE_TIME_MS   60000UL

float idleMin = 1e9;
float idleMax = -1e9;
unsigned long idleStartTime = 0;
bool idleActive = false;

// HX event control
bool hxCaptureActive = false;
bool centrePending = false;
bool eventReady = false;

// 🔑 HX GATE
#define HX_GATE_THRESHOLD 40.0
bool hxGateOpen = false;

float lastPressurePeak = 0.0;
float currentHXValue = 0.0;

// HX timeout
unsigned long hxCaptureStartTime = 0;
#define HX_CAPTURE_TIMEOUT_MS 1200

// =================================================
// FSR CONFIG
// =================================================
const int fsrPins[3] = {34, 35, 32};   // Right, Centre, Left
const float FORCE_THRESHOLD[3] = {90000, 5000, 4000};

const float Vcc = 3.3;
const float Rfixed = 47000.0;
float K[3] = {150000.0, 150000.0, 150000.0};
float zeroOffsetFSR[3] = {0, 0, 0};

const int NUM_SAMPLES = 20;

bool systemLocked = false;
String currentStatus = "NONE";
String lastStatus = "NONE";
int centreCount = 0;

// =================================================
// HX711 FUNCTIONS
// =================================================
void recalibrateZeroHX711() {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    while (!scale.is_ready());
    sum += scale.read();
  }
  zero_offset = sum / 20;
}

float readHX711Value() {
  long raw = scale.read();
  float value = (raw - zero_offset) / calibration_factor;
  return abs(value);
}

void handleHX711Peak(float value) {

  if (!hxCaptureActive) return;

  // 🔑 Gate opens ONLY after threshold
  if (!hxGateOpen) {
    if (value >= HX_GATE_THRESHOLD) {
      hxGateOpen = true;
      peakValue = value;
      lastValue = value;
      fallCount = 0;
      Serial.println("🔓 HX GATE OPENED");
    }
    return; // ignore until gate opens
  }

  // Normal peak detection AFTER gate
  if (value > peakValue) {
    peakValue = value;
    fallCount = 0;
  }
  else if (value < lastValue - NOISE_THRESH) {
    fallCount++;
    if (fallCount >= FALL_CONFIRM) {

      lastPressurePeak = peakValue;

      Serial.print(">>> HX PEAK: ");
      Serial.println(lastPressurePeak, 2);

      hxCaptureActive = false;
      centrePending = false;
      eventReady = true;
      centreCount++;

      peakValue = 0;
      fallCount = 0;
    }
  }

  lastValue = value;
}

// =================================================
// FSR FUNCTIONS (UNCHANGED)
// =================================================
float readAveragedADC(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(pin);
  return sum / (float)NUM_SAMPLES;
}

void calibrateFSR() {
  delay(2000);
  for (int i = 0; i < 3; i++) {
    float adc = readAveragedADC(fsrPins[i]);
    float voltage = (adc / 4095.0) * Vcc;
    if (voltage > 0.01) {
      float Rsensor = Rfixed * ((Vcc / voltage) - 1.0);
      zeroOffsetFSR[i] = 1.0 / Rsensor;
    }
  }
}

void handleFSR() {

  float forces[3] = {0, 0, 0};
  currentStatus = "NONE";

  for (int i = 0; i < 3; i++) {
    float adc = readAveragedADC(fsrPins[i]);
    float voltage = (adc / 4095.0) * Vcc;

    if (voltage > 0.05) {
      float Rsensor = Rfixed * ((Vcc / voltage) - 1.0);
      float conductance = (1.0 / Rsensor) - zeroOffsetFSR[i];
      if (conductance < 0) conductance = 0;
      forces[i] = conductance * K[i];
    }
  }

  if (!systemLocked) {
    if (forces[1] > FORCE_THRESHOLD[1]) {
      currentStatus = "CENTRE";
      systemLocked = true;
    }
  }
  else {
    if (forces[0] < FORCE_THRESHOLD[0] &&
        forces[1] < FORCE_THRESHOLD[1] &&
        forces[2] < FORCE_THRESHOLD[2]) {
      systemLocked = false;
    }
  }
}

// =================================================
// SETUP
// =================================================
void setup() {
  Serial.begin(115200);
  scale.begin(HX711_DOUT, HX711_SCK);
  recalibrateZeroHX711();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  calibrateFSR();
}

// =================================================
// LOOP
// =================================================
unsigned long lastFSRTime = 0;
unsigned long lastPrint = 0;

void loop() {

  // HX
  if (scale.is_ready()) {
    float v = readHX711Value();
    currentHXValue = v;
    handleHX711Peak(v);
  }

  // FSR
  if (millis() - lastFSRTime > 25) {
    lastFSRTime = millis();
    handleFSR();
  }

  // CENTRE trigger
  if (currentStatus == "CENTRE" && lastStatus != "CENTRE") {
    if (!centrePending) {
      centrePending = true;
      hxCaptureActive = true;
      hxGateOpen = false;
      peakValue = 0;
      fallCount = 0;
      hxCaptureStartTime = millis();
      Serial.println("🎯 CENTRE → WAITING FOR HX >= 40");
    }
  }

  // Timeout discard if gate never opened
  if (hxCaptureActive &&
      !hxGateOpen &&
      millis() - hxCaptureStartTime > HX_CAPTURE_TIMEOUT_MS) {

    Serial.println("❌ HX NEVER CROSSED 40 → EVENT DISCARDED");

    hxCaptureActive = false;
    centrePending = false;
    hxGateOpen = false;
  }

  // JSON
  if (eventReady) {
    String json = "{";
    json += "\"pressure\":" + String(lastPressurePeak, 2) + ",";
    json += "\"status\":\"CENTRE\",";
    json += "\"count\":" + String(centreCount);
    json += "}";
    Serial.println(json);
    eventReady = false;
  }

  // Debug print
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.print("STATE=");
    Serial.print(currentStatus);
    Serial.print(" HX=");
    Serial.print(currentHXValue, 2);
    Serial.print(" COUNT=");
    Serial.println(centreCount);
  }

  lastStatus = currentStatus;
}
