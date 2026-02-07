const int PIEZO_L = 33;
const int PIEZO_C = 35;
const int PIEZO_R = 32;

void setup() {
  Serial.begin(115200);

  analogReadResolution(12); // 0–4095
  analogSetPinAttenuation(PIEZO_L, ADC_11db);
  analogSetPinAttenuation(PIEZO_C, ADC_11db);
  analogSetPinAttenuation(PIEZO_R, ADC_11db);
}

void loop() {
  int L = analogRead(PIEZO_L);
  int C = analogRead(PIEZO_C);
  int R = analogRead(PIEZO_R);

  Serial.print("L=");
  Serial.print(L);
  Serial.print("  C=");
  Serial.print(C);
  Serial.print("  R=");
  Serial.println(R);

  delay(50);
}
