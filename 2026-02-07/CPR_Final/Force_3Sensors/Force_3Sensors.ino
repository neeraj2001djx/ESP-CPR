// =========================
// SETTINGS
// =========================

// Separate thresholds for each sensor
const float FORCE_THRESHOLD[3] = {90000, 5000, 4000};

bool systemLocked = false;

// =========================
// Pin Definitions
// =========================

const int fsrPins[3] = {34, 35, 32};

const float Vcc = 3.3;
const float Rfixed = 47000.0;

float K[3] = {150000.0, 150000.0, 150000.0};
float zeroOffset[3] = {0, 0, 0};

const int NUM_SAMPLES = 200;

// =========================
// Read Averaged ADC
// =========================

float readAveragedADC(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(500);
  }
  return sum / (float)NUM_SAMPLES;
}

// =========================
// Calibration
// =========================

void calibrateAll() {

  Serial.println("Calibrating...");
  delay(2000);

  for (int i = 0; i < 3; i++) {

    float adc = readAveragedADC(fsrPins[i]);
    float voltage = (adc / 4095.0) * Vcc;

    if (voltage > 0.01) {
      float Rsensor = Rfixed * ((Vcc / voltage) - 1.0);
      zeroOffset[i] = 1.0 / Rsensor;
    } else {
      zeroOffset[i] = 0;
    }
  }

  Serial.println("Calibration Done.");
  Serial.println();
}

// =========================
// Setup
// =========================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  calibrateAll();
}

// =========================
// Main Loop
// =========================

void loop() {

  float forces[3] = {0, 0, 0};

  // ===== Read Sensors =====
  for (int i = 0; i < 3; i++) {

    float adc = readAveragedADC(fsrPins[i]);
    float voltage = (adc / 4095.0) * Vcc;

    if (voltage > 0.05) {

      float Rsensor = Rfixed * ((Vcc / voltage) - 1.0);
      float conductance = 1.0 / Rsensor;

      conductance -= zeroOffset[i];

      if (conductance < 0)
        conductance = 0;

      float force = conductance * K[i];

      if (force < 0.2)
        force = 0;

      forces[i] = force;
    }
  }

  // =========================
  // CHECK RELEASE (Unlock Logic)
  // =========================

  if (systemLocked) {

    bool allBelow = true;

    for (int i = 0; i < 3; i++) {
      if (forces[i] >= FORCE_THRESHOLD[i]) {
        allBelow = false;
        break;
      }
    }

    if (allBelow) {
      systemLocked = false;
      Serial.println("System Ready");
    }
  }

  // =========================
  // NORMAL OPERATION
  // =========================

  if (!systemLocked) {

    // Print real forces
    for (int i = 0; i < 3; i++) {
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(" Force(N): ");
      Serial.print(forces[i], 2);
      Serial.print("   ");
    }
    Serial.println();

    int activeSensor = -1;
    float maxForce = 0;

    // Check against individual thresholds
    for (int i = 0; i < 3; i++) {

      if (forces[i] > FORCE_THRESHOLD[i] && forces[i] > maxForce) {
        maxForce = forces[i];
        activeSensor = i;
      }
    }

    if (activeSensor == 0) {
      Serial.println("Status = Right");
      systemLocked = true;
    }
    else if (activeSensor == 1) {
      Serial.println("Status = Centre");
      systemLocked = true;
    }
    else if (activeSensor == 2) {
      Serial.println("Status = Left");
      systemLocked = true;
    }
  }

  delay(50);
}
