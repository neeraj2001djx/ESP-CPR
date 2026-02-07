#define TRIG_PIN 5
#define ECHO_PIN 18

float zeroDistance = 0;

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20 ms timeout
  if (duration == 0) return -1;

  return duration * 0.01715; // cm
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

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  delay(1000); // let sensor settle

  // AUTO-ZERO
  zeroDistance = readStableDistance();

  Serial.print("Zero distance set to: ");
  Serial.print(zeroDistance);
  Serial.println(" cm");
}

void loop() {
  float currentDistance = readDistanceCM();
  if (currentDistance < 0) return;

  float depth = zeroDistance - currentDistance;

  if (depth < 0) depth = 0; // no negative depth

  Serial.print("CPR Depth: ");
  Serial.print(depth);
  Serial.println(" cm");

  delay(30); // ~33 Hz
}
