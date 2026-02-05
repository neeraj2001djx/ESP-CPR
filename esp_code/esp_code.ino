#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

/* ================= WIFI ================= */
const char* ssid = "Digital_Spectra_2.4G";
const char* password = "digital@9711";

/* ============ WEBSOCKET SERVER ============ */
WebSocketsServer webSocket(82);

/* ============ SENSOR PINS ============ */
#define FSR_L 32
#define FSR_C 35
#define FSR_R 34
#define ENDSTOP_PIN 25

/* ============ TUNING PARAMETERS ============ */
#define PRESS_THRESHOLD     100
#define INC_THRESHOLD       200
#define SMOOTH_ALPHA        0.2

/* ---- SIDE LOGIC ---- */
#define SIDE_RATIO_CENTRE   0.75f   // >= this → centre
#define DOM_THRESHOLD_RIGHT 250
#define DOM_THRESHOLD_LEFT  400     // LEFT less sensitive

/* ============ ANTI-FLICKER ============ */
#define STATE_HOLD_OTHER    250
#define STATE_HOLD_LEFT     250

/* ============ GLOBALS ============ */
int baseL = 0, baseC = 0, baseR = 0;
float smoothPressure = 0;

int pressCount = 0;
bool lastEndstopState = HIGH;

String currentStatus = "none";
String candidateStatus = "none";
unsigned long stateChangeTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  /* ---------- WIFI ---------- */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
  WiFi.setSleep(false);

  /* ---------- WEBSOCKET ---------- */
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  /* ---------- ADC ---------- */
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  /* ---------- CALIBRATION ---------- */
  Serial.println("Calibrating (do NOT touch sensors)");
  unsigned long t0 = millis();

  while (millis() - t0 < 2000) {
    int vL = analogRead(FSR_L);
    int vC = analogRead(FSR_C);
    int vR = analogRead(FSR_R);

    if (vL > baseL) baseL = vL;
    if (vC > baseC) baseC = vC;
    if (vR > baseR) baseR = vR;

    delay(10);
  }

  Serial.printf("BASE → L:%d C:%d R:%d\n", baseL, baseC, baseR);
}

void loop() {
  webSocket.loop();

  static unsigned long lastSample = 0;
  if (millis() - lastSample < 100) return;
  lastSample = millis();

  /* ---------- READ ---------- */
  int fL = analogRead(FSR_L);
  int fC = analogRead(FSR_C);
  int fR = analogRead(FSR_R);

  int nL = fL - baseL;
  int nC = fC - baseC;
  int nR = fR - baseR;

  /* ---------- PRESSURE ---------- */
  int pressure = abs(nL) + abs(nC) + abs(nR);
  pressure = constrain(pressure / 10, 0, 1000);

  smoothPressure =
    (1.0 - SMOOTH_ALPHA) * smoothPressure +
     SMOOTH_ALPHA * pressure;
  pressure = (int)smoothPressure;

  /* ---------- ENDSTOP ---------- */
  bool endstopState = digitalRead(ENDSTOP_PIN);
  bool endstopPressed = (endstopState == LOW);

  if (lastEndstopState == HIGH && endstopState == LOW) {
    pressCount++;
  }
  lastEndstopState = endstopState;

  /* ---------- RESET ON RELEASE ---------- */
  if (!endstopPressed) {
    currentStatus = "none";
    candidateStatus = "none";
    stateChangeTime = 0;
  }

  /* ---------- DIRECTION (ONLY WHEN PRESSED) ---------- */
  if (endstopPressed && pressure > PRESS_THRESHOLD) {

    String detected = "none";

    int absL = abs(nL);
    int absR = abs(nR);

    float sideRatio = (float)min(absL, absR) /
                      (float)max(absL, absR);

    bool centreActive = nC > INC_THRESHOLD;

    /* ---------- STRICT CENTRE ---------- */
    if (centreActive && sideRatio >= SIDE_RATIO_CENTRE) {
      detected = "centre";
    }
    /* ---------- LEFT ---------- */
    else if ((absL - absR) > DOM_THRESHOLD_LEFT &&
             sideRatio < SIDE_RATIO_CENTRE) {
      detected = "left";
    }
    /* ---------- RIGHT ---------- */
    else if ((absR - absL) > DOM_THRESHOLD_RIGHT &&
             sideRatio < SIDE_RATIO_CENTRE) {
      detected = "right";
    }

    /* ---------- ANTI-FLICKER ---------- */
    if (detected != currentStatus) {

      if (detected != candidateStatus) {
        candidateStatus = detected;
        stateChangeTime = millis();
      }

      unsigned long holdTime =
        (candidateStatus == "left") ?
        STATE_HOLD_LEFT : STATE_HOLD_OTHER;

      if (millis() - stateChangeTime > holdTime) {
        currentStatus = candidateStatus;
      }
    }
  }

  /* ---------- SEND ---------- */
  StaticJsonDocument<256> doc;
  doc["pressure"] = pressure;
  doc["count"] = pressCount;
  doc["status"] = currentStatus;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  webSocket.broadcastTXT(buffer, len);

  /* ---------- DEBUG ---------- */
  float dbgRatio = (float)min(abs(nL), abs(nR)) /
                   (float)max(abs(nL), abs(nR));

  Serial.printf(
    "nL:%d nC:%d nR:%d | P:%d | ratio:%.2f | %s | end:%d\n",
    nL, nC, nR, pressure,
    dbgRatio,
    currentStatus.c_str(),
    endstopPressed
  );
}

/* ---------- WS EVENTS ---------- */
void webSocketEvent(uint8_t, WStype_t, uint8_t*, size_t) {}
