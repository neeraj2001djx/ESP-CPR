// ================= PIEZO PINS =================
const int PIEZO_L = 26;
const int PIEZO_C = 35;
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

// ================= ULTRASONIC FUNCTIONS =================
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

  delay(1000);
  zeroDistance = readStableDistance();
}

// ================= LOOP =================
void loop() {

  // -------- PIEZO READ --------
  int L = analogRead(PIEZO_L);
  int C = analogRead(PIEZO_C);
  int R = analogRead(PIEZO_R);

  // -------- START COLLECTION --------
  if (!collecting && status == "NONE" &&
      (L > PIEZO_THRESHOLD || C > PIEZO_THRESHOLD || R > PIEZO_THRESHOLD)) {
    collecting = true;
    collectCount = 0;
    sumL = sumC = sumR = 0;
  }

  // -------- COLLECT WINDOW --------
  if (collecting) {
    sumL += L;
    sumC += C;
    sumR += R;
    collectCount++;

    if (collectCount >= PIEZO_WINDOW) {
      if (sumL > sumC && sumL > sumR) {
        status = "LEFT";
      }
      else if (sumR > sumL && sumR > sumC) {
        status = "RIGHT";
      }
      else {
        status = "CENTER";
      }

      collecting = false;   // stop collecting
    }
  }

  // -------- ULTRASONIC READ --------
  float currentDistance = readDistanceCM();
  float depth = 0;

  if (currentDistance > 0) {
    depth = zeroDistance - currentDistance;
    if (depth < 0) depth = 0;
  }

  // -------- SERIAL OUTPUT (ALWAYS) --------
  Serial.print("Depth=");
  Serial.print(depth);
  Serial.print(" cm  |  Status=");
  Serial.println(status);

  // -------- AUTO RESET STATUS --------
  if (status != "NONE") {
    status = "NONE";  // reset after one print
  }

  delay(50);
}
