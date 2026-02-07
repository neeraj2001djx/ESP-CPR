#include <WiFi.h>
#include <WebSocketsServer.h>

// =========================
// WIFI SETTINGS
// =========================
const char* ssid = "Digital_Spectra_2.4G";
const char* password = "digital@9711";

WebSocketsServer webSocket = WebSocketsServer(81);

// =========================
// FSR SETTINGS
// =========================
const float FORCE_THRESHOLD[3] = {90000, 5000, 4000};
bool systemLocked = false;

const int fsrPins[3] = {34, 35, 32};

const float Vcc = 3.3;
const float Rfixed = 47000.0;

float K[3] = {150000.0, 150000.0, 150000.0};
float zeroOffset[3] = {0, 0, 0};

const int NUM_SAMPLES = 50;

// =========================
// ULTRASONIC SETTINGS
// =========================
#define TRIG_PIN 5
#define ECHO_PIN 18

float zeroDistance = 0;

// =========================
// GLOBAL
// =========================
String currentStatus = "NONE";

// 🔥 ADDED FOR COUNT
int centreCount = 0;
String lastStatus = "NONE";

// ==========================================================
// FUNCTIONS
// ==========================================================

float readAveragedADC(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return sum / (float)NUM_SAMPLES;
}

void calibrateAll() {
  Serial.println("Calibrating FSR...");
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

  Serial.println("FSR Calibration Done.");
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  if (duration == 0) return -1;

  return duration * 0.01715;
}

float readStableDistance() {
  float sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    float d = readDistanceCM();
    if (d > 0) {
      sum += d;
      count++;
    }
    delay(20);
  }

  if (count == 0) return -1;
  return sum / count;
}

// ==========================================================
// WEBSOCKET EVENTS
// ==========================================================

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("Client %u connected\n", num);
  }
  else if (type == WStype_DISCONNECTED) {
    Serial.printf("Client %u disconnected\n", num);
  }
}

// ==========================================================
// SETUP
// ==========================================================

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  calibrateAll();

  zeroDistance = readStableDistance();
  Serial.print("Zero Distance: ");
  Serial.println(zeroDistance);

  Serial.println("System Ready\n");
}

// ==========================================================
// LOOP
// ==========================================================

void loop() {

  webSocket.loop();

  float forces[3] = {0, 0, 0};
  currentStatus = "NONE";

  // ===== Read FSR =====
  for (int i = 0; i < 3; i++) {

    float adc = readAveragedADC(fsrPins[i]);
    float voltage = (adc / 4095.0) * Vcc;

    if (voltage > 0.05) {

      float Rsensor = Rfixed * ((Vcc / voltage) - 1.0);
      float conductance = 1.0 / Rsensor;

      conductance -= zeroOffset[i];
      if (conductance < 0) conductance = 0;

      float force = conductance * K[i];
      if (force < 0.2) force = 0;

      forces[i] = force;
    }
  }

  // ===== Unlock Logic =====
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
    }
  }

  // ===== Detection =====
  if (!systemLocked) {

    int activeSensor = -1;
    float maxForce = 0;

    for (int i = 0; i < 3; i++) {
      if (forces[i] > FORCE_THRESHOLD[i] && forces[i] > maxForce) {
        maxForce = forces[i];
        activeSensor = i;
      }
    }

    if (activeSensor == 0) {
      currentStatus = "RIGHT";
      systemLocked = true;
    }
    else if (activeSensor == 1) {
      currentStatus = "CENTRE";
      systemLocked = true;
    }
    else if (activeSensor == 2) {
      currentStatus = "LEFT";
      systemLocked = true;
    }
  }

  // ===== COUNT LOGIC (ADDED ONLY THIS PART) =====
  if (currentStatus == "CENTRE" && lastStatus != "CENTRE") {
    centreCount++;
  }

  lastStatus = currentStatus;

  // ===== Ultrasonic =====
  float currentDistance = readDistanceCM();
  float depth = 0;

  if (currentDistance > 0) {
    depth = zeroDistance - currentDistance;
    if (depth < 0) depth = 0;
  }

  // ===== JSON Packet =====
  String json = "{";
  json += "\"pressure\":" + String(depth, 2) + ",";
  json += "\"status\":\"" + currentStatus + "\",";
  json += "\"count\":" + String(centreCount);
  json += "}";

  Serial.println(json);
  webSocket.broadcastTXT(json);

  delay(40);
}
