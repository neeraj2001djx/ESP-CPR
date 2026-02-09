#include <WiFi.h>

const char* ssid = "Mi Router 4A";
const char* password = "Wasd$2123";

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n==============================");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("==============================");
}

void loop() {
}
