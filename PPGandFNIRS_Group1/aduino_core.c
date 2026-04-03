/*
  Final dual-wavelength optical biosensor code
  Arduino Mega

  Features:
  - Ambient-light rejection
  - Dual-wavelength acquisition
  - Pulse extraction from DET2
  - Adaptive LED intensity control (auto-gain)

  Pin map:
    D10 -> 730 nm LED driver EN/PWM
    D11 -> 850 nm LED driver EN/PWM
    A0  -> DET1
    A1  -> DET2

  Serial output:
    ms,det1_730,det2_730,det1_850,det2_850,pulse730_det2,pulse850_det2
*/

const int LED730 = 10;
const int LED850 = 11;

const int DET1 = A0;
const int DET2 = A1;

// Start values for LED brightness
int PWM730 = 120;
int PWM850 = 120;

// ADC averaging
const int SAMPLES = 12;
const unsigned int SETTLE_US = 2000;

// Pulse baseline tracking
const float BASELINE_ALPHA = 0.02f;
float baseline730_det2 = 0.0f;
float baseline850_det2 = 0.0f;
bool baselineInitialized = false;

// Auto-gain target and gain
const int TARGET_ADC = 600;     // desired detector level
const float GAIN = 0.01f;       // control aggressiveness

// PWM limits
const int PWM_MIN = 10;
const int PWM_MAX = 255;

int readAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / SAMPLES;
}

void ledsOff() {
  analogWrite(LED730, 0);
  analogWrite(LED850, 0);
}

void setup() {
  pinMode(LED730, OUTPUT);
  pinMode(LED850, OUTPUT);

  ledsOff();

  Serial.begin(115200);
  delay(1000);

  Serial.println("ms,det1_730,det2_730,det1_850,det2_850,pulse730_det2,pulse850_det2");
}

void loop() {
  int amb1a, amb2a;
  int amb1b, amb2b;
  int on730_1, on730_2;
  int on850_1, on850_2;

  // ------------------------------------------------
  // 1) Ambient sample before 730
  // ------------------------------------------------
  ledsOff();
  delayMicroseconds(SETTLE_US);
  amb1a = readAverage(DET1);
  amb2a = readAverage(DET2);

  // ------------------------------------------------
  // 2) 730 nm measurement
  // ------------------------------------------------
  analogWrite(LED730, PWM730);
  delayMicroseconds(SETTLE_US);
  on730_1 = readAverage(DET1);
  on730_2 = readAverage(DET2);
  analogWrite(LED730, 0);

  // ------------------------------------------------
  // 3) Ambient sample before 850
  // ------------------------------------------------
  delayMicroseconds(SETTLE_US);
  amb1b = readAverage(DET1);
  amb2b = readAverage(DET2);

  // ------------------------------------------------
  // 4) 850 nm measurement
  // ------------------------------------------------
  analogWrite(LED850, PWM850);
  delayMicroseconds(SETTLE_US);
  on850_1 = readAverage(DET1);
  on850_2 = readAverage(DET2);
  analogWrite(LED850, 0);

  // ------------------------------------------------
  // 5) Average ambient
  // ------------------------------------------------
  int amb1 = (amb1a + amb1b) / 2;
  int amb2 = (amb2a + amb2b) / 2;

  // ------------------------------------------------
  // 6) Ambient-subtracted optical signals
  // ------------------------------------------------
  int net730_1 = on730_1 - amb1;
  int net730_2 = on730_2 - amb2;
  int net850_1 = on850_1 - amb1;
  int net850_2 = on850_2 - amb2;

  if (net730_1 < 0) net730_1 = 0;
  if (net730_2 < 0) net730_2 = 0;
  if (net850_1 < 0) net850_1 = 0;
  if (net850_2 < 0) net850_2 = 0;

  // ------------------------------------------------
  // 7) Initialize pulse baselines once
  // ------------------------------------------------
  if (!baselineInitialized) {
    baseline730_det2 = net730_2;
    baseline850_det2 = net850_2;
    baselineInitialized = true;
  }

  // ------------------------------------------------
  // 8) Slow baseline tracking for pulse extraction
  // ------------------------------------------------
  baseline730_det2 = (1.0f - BASELINE_ALPHA) * baseline730_det2 + BASELINE_ALPHA * net730_2;
  baseline850_det2 = (1.0f - BASELINE_ALPHA) * baseline850_det2 + BASELINE_ALPHA * net850_2;

  float pulse730_det2 = net730_2 - baseline730_det2;
  float pulse850_det2 = net850_2 - baseline850_det2;

  // ------------------------------------------------
  // 9) Adaptive LED intensity control
  //    Uses DET2 because it is your stronger channel
  // ------------------------------------------------
  int error730 = TARGET_ADC - net730_2;
  int error850 = TARGET_ADC - net850_2;

  PWM730 += (int)(GAIN * error730);
  PWM850 += (int)(GAIN * error850);

  PWM730 = constrain(PWM730, PWM_MIN, PWM_MAX);
  PWM850 = constrain(PWM850, PWM_MIN, PWM_MAX);

  // ------------------------------------------------
  // 10) Serial output for Python
  // ------------------------------------------------
  Serial.print(millis());
  Serial.print(",");

  Serial.print(net730_1);
  Serial.print(",");
  Serial.print(net730_2);
  Serial.print(",");
  Serial.print(net850_1);
  Serial.print(",");
  Serial.print(net850_2);
  Serial.print(",");

  Serial.print(pulse730_det2, 2);
  Serial.print(",");
  Serial.println(pulse850_det2, 2);

  delay(1);  

