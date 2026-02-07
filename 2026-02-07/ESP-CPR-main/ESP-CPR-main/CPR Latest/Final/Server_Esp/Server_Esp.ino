#include <WiFi.h>
#include <WebSocketsServer.h>

// ================= WIFI =================
const char* ssid = "Digital_Spectra_2.4G";
const char* password = "digital@9711";

// ================= WEBSOCKET =================
WebSocketsServer webSocket = WebSocketsServer(81);

// ================= PIEZO PINS (ADC1 ONLY) =================
const int PIEZO_L = 35;
const int PIEZO_C = 32;
const int PIEZO_R = 33;

// ================= ULTRASONIC PINS =================
#define TRIG_PIN 5
#define ECHO_PIN 18

float zeroDistance = 0;

// ================= PIEZO LOGIC =================
const int PIEZO_THRESHOLD = 150;
const int PIEZO_WINDOW = 5;

bool collecting = false;
int collectCount = 0;

long sumL = 0, sumC = 0, sumR = 0;
String status = "NONE";

// ================= ULTRASONIC =================
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

// ================= WEBSOCKET EVENT =================
void webSocketEvent(uint8_t num, WStype_t type,
                    uint8_t * payload, size_t length) {
  // no incoming commands
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIEZO_L, ADC_11db);
  analogSetPinAttenuation(PIEZO_C, ADC_11db);
  analogSetPinAttenuation(PIEZO_R, ADC_11db);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  delay(1000);
  zeroDistance = readStableDistance();

  Serial.print("Zero distance = ");
  Serial.print(zeroDistance);
  Serial.println(" cm");
}

// ================= LOOP =================
void loop() {

  webSocket.loop();

  // -------- PIEZO READ --------
  int L = analogRead(PIEZO_L);
  int C = analogRead(PIEZO_C);
  int R = analogRead(PIEZO_R);

  // -------- PIEZO LOGIC --------
  if (!collecting && status == "NONE" &&
      (L > PIEZO_THRESHOLD || C > PIEZO_THRESHOLD || R > PIEZO_THRESHOLD)) {
    collecting = true;
    collectCount = 0;
    sumL = sumC = sumR = 0;
  }

  if (collecting) {
    sumL += L;
    sumC += C;
    sumR += R;
    collectCount++;

    if (collectCount >= PIEZO_WINDOW) {
      if (sumL > sumC && sumL > sumR) status = "LEFT";
      else if (sumR > sumL && sumR > sumC) status = "RIGHT";
      else status = "CENTER";

      collecting = false;
    }
  }

  // -------- ULTRASONIC --------
  float depth = 0;
  float d = readDistanceCM();
  if (d > 0) {
    depth = zeroDistance - d;
    if (depth < 0) depth = 0;
  }

  // -------- SERIAL OUTPUT --------
  Serial.print("L=");
  Serial.print(L);
  Serial.print("  C=");
  Serial.print(C);
  Serial.print("  R=");
  Serial.print(R);
  Serial.print("  | Depth=");
  Serial.print(depth, 2);
  Serial.print(" cm  | Status=");
  Serial.println(status);

  // -------- SEND DATA (JSON) --------
  String msg = "{";
  msg += "\"depth\":" + String(depth, 2) + ",";
  msg += "\"status\":\"" + status + "\"";
  msg += "}";

  webSocket.broadcastTXT(msg);

  // reset status after send
  if (status != "NONE") status = "NONE";

  delay(50); // ~20 Hz
}
