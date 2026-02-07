#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <math.h>

/* ================= WIFI ================= */
const char* ssid = "Digital_Spectra_2.4G";
const char* password = "digital@9711";

/* ============ WEBSOCKET SERVER ============ */
WebSocketsServer webSocket(82);

/* ============ PINS ============ */
#define FSR_L   32
#define FSR_C   35
#define FSR_R   33
#define IR_PIN  34

/* ============ SETTINGS ============ */
#define SMOOTH_ALPHA 0.2
const int irSamples = 20;
const float irDeadZone = 0.1;

/* ============ GLOBALS ============ */

// FSR
int baseL = 0;
int baseC = 0;
int baseR = 0;

float smoothL = 0;
float smoothC = 0;
float smoothR = 0;

// IR
float zeroDistance = 0;
float filteredADC = 0;

/* ================= UTIL ================= */

float readAveragedADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / (float)samples;
}

float adcToVoltage(float adcValue) {
  return adcValue * (3.3 / 4095.0);
}

float voltageToDistance(float voltage) {
  if (voltage <= 0.1) return 80;
  return 29.988 * pow(voltage, -1.173);
}

/* ================= CALIBRATION ================= */

void calibrateFSR() {
  Serial.println("Calibrating FSRs... Do NOT touch");

  long sumL = 0, sumC = 0, sumR = 0;

  for (int i = 0; i < 200; i++) {
    sumL += analogRead(FSR_L);
    sumC += analogRead(FSR_C);
    sumR += analogRead(FSR_R);
    delay(5);
  }

  baseL = sumL / 200;
  baseC = sumC / 200;
  baseR = sumR / 200;

  Serial.println("FSR Calibration Done");
}

void calibrateIR() {
  Serial.println("Calibrating IR zero...");
  delay(1000);

  filteredADC = readAveragedADC(IR_PIN, irSamples);
  zeroDistance = voltageToDistance(adcToVoltage(filteredADC));

  Serial.print("Zero distance = ");
  Serial.print(zeroDistance);
  Serial.println(" cm");
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  /* WIFI */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
  WiFi.setSleep(false);

  /* WEBSOCKET */
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  /* CALIBRATION */
  calibrateFSR();
  calibrateIR();

  smoothL = 0;
  smoothC = 0;
  smoothR = 0;
}

/* ================= LOOP ================= */

void loop() {

  webSocket.loop();

  static unsigned long lastSample = 0;
  if (millis() - lastSample < 100) return;
  lastSample = millis();

  /* ---------- FSR READ ---------- */
  int rawL = analogRead(FSR_L) - baseL;
  int rawC = analogRead(FSR_C) - baseC;
  int rawR = analogRead(FSR_R) - baseR;

  if (rawL < 0) rawL = 0;
  if (rawC < 0) rawC = 0;
  if (rawR < 0) rawR = 0;

  /* ---------- SMOOTH ---------- */
  smoothL = (1.0 - SMOOTH_ALPHA) * smoothL + SMOOTH_ALPHA * rawL;
  smoothC = (1.0 - SMOOTH_ALPHA) * smoothC + SMOOTH_ALPHA * rawC;
  smoothR = (1.0 - SMOOTH_ALPHA) * smoothR + SMOOTH_ALPHA * rawR;

  /* ---------- IR READ ---------- */
  float rawIR = readAveragedADC(IR_PIN, irSamples);
  filteredADC = filteredADC * 0.85 + rawIR * 0.15;

  float distance = voltageToDistance(adcToVoltage(filteredADC));
  float relative = distance - zeroDistance;

  if (abs(relative) < irDeadZone)
    relative = 0;

  /* ---------- JSON ---------- */
  StaticJsonDocument<256> doc;
  doc["f1"] = (int)smoothL;
  doc["f2"] = (int)smoothC;
  doc["Left"] = (int)smoothR;
  doc["distance_cm"] = distance;
  doc["relative_cm"] = relative;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);

  webSocket.broadcastTXT(buffer, len);
  Serial.println(buffer);
}

/* ---------- WS EVENT ---------- */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // currently unused
}
