#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>

// ---------------- SERIAL ----------------
#define SerialMon Serial

// ---------------- EMERGENCY CONTACTS ----------------
const char* EMERGENCY_CONTACTS[] = {
  "+91XXXXXXXXXX",   // Primary contact
  "+91XXXXXXXXXX",   // Secondary contact
  "+91XXXXXXXXXX"    // Tertiary contact
};
const int NUM_CONTACTS = sizeof(EMERGENCY_CONTACTS) / sizeof(EMERGENCY_CONTACTS[0]);

// ---------------- PIN CONFIGURATION ----------------
#define MODEM_TX 17
#define MODEM_RX 18
#define MODEM_RST 4

#define GPS_TX_PIN 26
#define GPS_RX_PIN 27

#define BUZZER_PIN 25
#define CANCEL_BUTTON_PIN 32
#define HELMET_WORN_PIN 33   // TTP223 (HIGH when touched)

#define SDA_PIN 21
#define SCL_PIN 22

// ---------------- MPU6050 ----------------
const uint8_t MPU_ADDR = 0x68;

// ---------------- SYSTEM CONSTANTS ----------------
#define COUNTDOWN_TIME 15000
#define MODEM_BAUD 9600

// Crash detection tuning
float baseline_g = 1.0;
const float DELTA_THRESHOLD_G = 0.8;
const float LPF_ALPHA = 0.2;
const float HYSTERESIS_G = 0.15;

float delta_lpf = 0.0;
bool crashTriggered = false;
int consecutiveHits = 0;
const int REQUIRED_HITS = 1;

unsigned long lastAccelRead = 0;
const unsigned long ACCEL_INTERVAL_MS = 10;

// Helmet debounce
const unsigned long HELMET_DEBOUNCE_MS = 1000;
bool helmetRaw = false;
bool helmetStable = false;
unsigned long helmetLastChange = 0;

// Countdown
unsigned long countdownStart = 0;
unsigned long lastBeep = 0;

// ---------------- OBJECTS ----------------
HardwareSerial SerialAT(2);
HardwareSerial SerialGPS(1);

TinyGsm modem(SerialAT);
TinyGPSPlus gps;

// ---------------- SYSTEM STATES ----------------
enum SystemState {
  STATE_IDLE,
  STATE_MONITORING,
  STATE_COUNTDOWN,
  STATE_ALERT_SENT
};
SystemState currentState = STATE_IDLE;

// FUNCTION DECLARATIONS
void initGPS();
void initGSM();
void runBuzzer();
void sendEmergencyAlert();
void updateHelmetState();
bool detectCrash();
void resetCrashLogic();

bool readAccel(int16_t &ax, int16_t &ay, int16_t &az);
int readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t val);
float accelLSBperG();
void delayWithGPS(unsigned long ms);

// SETUP
void setup() {
  SerialMon.begin(115200);
  delay(200);
  SerialMon.println("=== SMART HELMET SYSTEM STARTED ===");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CANCEL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HELMET_WORN_PIN, INPUT_PULLDOWN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Wake MPU6050
  writeRegister(0x6B, 0x00);
  delay(50);

  initGPS();
  initGSM();
  resetCrashLogic();

  helmetRaw = digitalRead(HELMET_WORN_PIN);
  helmetStable = helmetRaw;
  helmetLastChange = millis();

  SerialMon.println("System Ready. Waiting for helmet...");
}

// LOOP
void loop() {
  while (SerialGPS.available()) gps.encode(SerialGPS.read());

  updateHelmetState();

  switch (currentState) {

    case STATE_IDLE:
      if (helmetStable) {
        SerialMon.println("Helmet worn → Monitoring");
        resetCrashLogic();
        currentState = STATE_MONITORING;
      }
      break;

    case STATE_MONITORING:
      if (!helmetStable) {
        SerialMon.println("Helmet removed → Idle");
        currentState = STATE_IDLE;
        break;
      }

      if (detectCrash()) {
        SerialMon.println("!!! CRASH DETECTED !!!");
        countdownStart = millis();
        lastBeep = millis();
        currentState = STATE_COUNTDOWN;
      }
      break;

    case STATE_COUNTDOWN:
      runBuzzer();

      if (digitalRead(CANCEL_BUTTON_PIN) == LOW) {
        digitalWrite(BUZZER_PIN, LOW);
        SerialMon.println("Alert cancelled by rider");
        resetCrashLogic();
        currentState = STATE_MONITORING;
      }

      if (millis() - countdownStart > COUNTDOWN_TIME) {
        digitalWrite(BUZZER_PIN, LOW);
        sendEmergencyAlert();
        resetCrashLogic();
        currentState = STATE_ALERT_SENT;
      }
      break;

    case STATE_ALERT_SENT:
      if (!helmetStable) {
        SerialMon.println("Helmet removed → Reset");
        currentState = STATE_IDLE;
      }
      delay(500);
      break;
  }
}

// HELMET DEBOUNCE
void updateHelmetState() {
  bool raw = digitalRead(HELMET_WORN_PIN);

  if (raw != helmetRaw) {
    helmetRaw = raw;
    helmetLastChange = millis();
  } else if (millis() - helmetLastChange > HELMET_DEBOUNCE_MS) {
    if (helmetStable != helmetRaw) {
      helmetStable = helmetRaw;
      SerialMon.print("Helmet state → ");
      SerialMon.println(helmetStable ? "WORN" : "REMOVED");
      if (helmetStable) resetCrashLogic();
    }
  }
}

// CRASH DETECTION (DELTA-G + LPF)
bool detectCrash() {
  if (millis() - lastAccelRead < ACCEL_INTERVAL_MS) return false;
  lastAccelRead = millis();

  int16_t ax, ay, az;
  if (!readAccel(ax, ay, az)) return false;

  float lsb = accelLSBperG();
  float x = ax / lsb;
  float y = ay / lsb;
  float z = az / lsb;

  float magnitude = sqrt(x*x + y*y + z*z);
  baseline_g = 0.98 * baseline_g + 0.02 * magnitude;

  float delta = fabs(magnitude - baseline_g);
  delta_lpf = LPF_ALPHA * delta + (1.0 - LPF_ALPHA) * delta_lpf;

  if (crashTriggered) return true;

  if (delta_lpf > DELTA_THRESHOLD_G) consecutiveHits++;
  else if (delta_lpf < (DELTA_THRESHOLD_G - HYSTERESIS_G)) consecutiveHits = 0;

  if (consecutiveHits >= REQUIRED_HITS) {
    crashTriggered = true;
    return true;
  }

  return false;
}

void resetCrashLogic() {
  crashTriggered = false;
  consecutiveHits = 0;
  delta_lpf = 0.0;
  baseline_g = 1.0;
}

// BUZZER
void runBuzzer() {
  if (millis() - lastBeep > 1000) digitalWrite(BUZZER_PIN, HIGH);
  if (millis() - lastBeep > 1500) {
    digitalWrite(BUZZER_PIN, LOW);
    lastBeep = millis();
  }
}

// GPS
void initGPS() {
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialMon.println("GPS initialized");
}

// GSM
void initGSM() {
  SerialMon.println("Initializing GSM...");
  pinMode(MODEM_RST, OUTPUT);

  digitalWrite(MODEM_RST, LOW);
  delay(3000);
  digitalWrite(MODEM_RST, HIGH);
  delay(5000);

  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  modem.init();
  modem.waitForNetwork(30000);
}

// EMERGENCY SMS
void sendEmergencyAlert() {
  String message;
  bool gpsFix = false;

  unsigned long t0 = millis();
  while (!gpsFix && millis() - t0 < 30000) {
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    if (gps.location.isValid()) {
      gpsFix = true;
      break;
    }
    delay(100);
  }

  if (gpsFix) {
    message = "ALERT: Accident detected. Location: https://maps.google.com/?q=" +
              String(gps.location.lat(), 6) + "," +
              String(gps.location.lng(), 6);
  } else {
    message = "ALERT: Accident detected. GPS location unavailable.";
  }

  for (int i = 0; i < NUM_CONTACTS; i++) {
    while (!modem.sendSMS(EMERGENCY_CONTACTS[i], message)) {
      delayWithGPS(5000);
    }
  }

  SerialMon.println("Emergency alerts sent.");
}

// LOW-LEVEL MPU6050
bool readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  return true;
}

float accelLSBperG() {
  return 8192.0; // ±4g range
}

int readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// HELPER
void delayWithGPS(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    delay(50);
  }
}
