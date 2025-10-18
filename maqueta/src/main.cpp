/*
  Maqueta control
  - 2 relés (IN1=D10, IN2=D11) controlan 2 luminarias (cada una 3 LEDs)
  - Stepper 28BYJ-48 con ULN2003, inputs D4..D7
  - 2 pulsadores: D2 = forward, D3 = back (hold to move)
  - Servo en D9 para ventilación
  - DHT22 en D8
  - MQ135 sensores en A0 y A1
  - Todo alimentado desde 5V Arduino (si consumo alto, usar fuente externa y compartir GND)

  Comportamiento:
  - Pulsador D2 presionado -> stepper gira forward; D3 presionado -> stepper gira backward
  - Relés controlables desde serial con '1' y '2' para toggle
  - MQ135 promedia lecturas y si supera umbral -> activa ventilación (servo)
  - Serial a 115200 para debug y estado
*/

#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// --- Hardware pins ---
const int RELAY1_PIN = 10; // luminaria A
const int RELAY2_PIN = 11; // luminaria B

// Relay logic: many 5V relay modules are active LOW (LOW energizes). Set true if your module is active LOW.
bool relayActiveLow = true;
bool relayState1 = false; // false = OFF
bool relayState2 = false;

// Apply relay state (on=true => turn relay ON). Handles active-low modules.
void applyRelay(int idx, bool on){
  int pin = (idx==0)?RELAY1_PIN:RELAY2_PIN;
  if (relayActiveLow) digitalWrite(pin, on?LOW:HIGH);
  else digitalWrite(pin, on?HIGH:LOW);
}

// Pulse a relay physically for diagnostic (blocks briefly)
void pulseRelayTest(int idx, unsigned int ms){
  int pin = (idx==0)?RELAY1_PIN:RELAY2_PIN;
  // set active
  if (relayActiveLow) digitalWrite(pin, LOW); else digitalWrite(pin, HIGH);
  Serial.print("[PulseTest] Relay"); Serial.print(idx+1); Serial.print(" set to active (pin state="); Serial.print(digitalRead(pin)); Serial.println(")");
  delay(ms);
  // restore inactive
  if (relayActiveLow) digitalWrite(pin, HIGH); else digitalWrite(pin, LOW);
  Serial.print("[PulseTest] Relay"); Serial.print(idx+1); Serial.print(" restored (pin state="); Serial.print(digitalRead(pin)); Serial.println(")");
}

const int BUTTON_FWD_PIN = 2; // pulsador adelante (active low)
const int BUTTON_BWD_PIN = 3; // pulsador atrás (active low)

const int STEPPER_PIN1 = 4;
const int STEPPER_PIN2 = 5;
const int STEPPER_PIN3 = 6;
const int STEPPER_PIN4 = 7;

const int SERVO_PIN = 9;
const int DHT_PIN = 8;
const int MQ1_PIN = A0;
const int MQ2_PIN = A1;

// --- DHT ---
#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

// --- Servo ---
Servo fanServo;
bool fanActive = false; // ventilación forzada

// --- Stepper ---
const uint8_t STEPSEQ[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
int stepIdx = 0;
unsigned long lastStepMs = 0;
unsigned long stepDelayMs = 7; // velocidad base

// --- Buttons debounce ---
unsigned long lastDebounceF = 0, lastDebounceB = 0;
const unsigned long DEBOUNCE_MS = 30;
int lastReadF = HIGH, lastReadB = HIGH;

// --- MQ logic (non-blocking EMA) ---
float mq1Avg = 0.0f, mq2Avg = 0.0f;
const float MQ_ALPHA = 0.25f; // EMA factor 0..1
const float MQ_THRESHOLD = 400.0f; // ADC units threshold (adjust to sensor)
unsigned long lastMqMs = 0;
const unsigned long MQ_INTERVAL_MS = 1000;
// fan non-blocking
unsigned long fanLastToggleMs = 0;
const unsigned long FAN_TOGGLE_MS = 400;
bool fanPos = false; // toggle state for servo positions

// --- INA219 (power monitoring) ---
Adafruit_INA219 ina219;
unsigned long lastInaMs = 0;
const unsigned long INA_INTERVAL_MS = 1000; // read every 1s
float inaBusV = 0.0f;
float inaCurrent_mA = 0.0f;
float inaPower_mW = 0.0f;
double energy_mWh = 0.0; // accumulated milliwatt-hours
bool inaInitialized = false;

// --- Helpers ---
void writeStepper(int idx){
  digitalWrite(STEPPER_PIN1, STEPSEQ[idx][0]);
  digitalWrite(STEPPER_PIN2, STEPSEQ[idx][1]);
  digitalWrite(STEPPER_PIN3, STEPSEQ[idx][2]);
  digitalWrite(STEPPER_PIN4, STEPSEQ[idx][3]);
}

void stepOnce(int dir){
  stepIdx = (stepIdx + (dir>0?1:7)) % 8;
  writeStepper(stepIdx);
}

// --- Setup ---
void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("Maqueta - inicio");

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  // ensure relays start OFF
  relayState1 = false; relayState2 = false;
  applyRelay(0, relayState1);
  applyRelay(1, relayState2);

  pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BWD_PIN, INPUT_PULLUP);

  pinMode(STEPPER_PIN1, OUTPUT); pinMode(STEPPER_PIN2, OUTPUT); pinMode(STEPPER_PIN3, OUTPUT); pinMode(STEPPER_PIN4, OUTPUT);
  writeStepper(0);

  pinMode(MQ1_PIN, INPUT); pinMode(MQ2_PIN, INPUT);

  dht.begin();

  // init INA219
  if (!ina219.begin()){
    Serial.println("INA219 not found. Check wiring.");
  } else {
    Serial.println("INA219 initialized");
    inaInitialized = true;
    // optional calibration depending on expected current range
    // ina219.setCalibration_32V_2A();
  }

  fanServo.attach(SERVO_PIN);
  fanServo.write(90); // neutral
  fanServo.detach(); // detach until needed

  // initialize MQ averages to first readings
  mq1Avg = analogRead(MQ1_PIN);
  mq2Avg = analogRead(MQ2_PIN);
  lastMqMs = millis();
}

// --- Loop ---
void loop(){
  unsigned long now = millis();

  // read buttons (simple hold to move)
  int rf = digitalRead(BUTTON_FWD_PIN);
  int rb = digitalRead(BUTTON_BWD_PIN);
  if (rf != lastReadF) { lastDebounceF = now; lastReadF = rf; }
  if (rb != lastReadB) { lastDebounceB = now; lastReadB = rb; }

  bool forward = false, backward = false;
  if ((now - lastDebounceF) > DEBOUNCE_MS) forward = (rf == LOW);
  if ((now - lastDebounceB) > DEBOUNCE_MS) backward = (rb == LOW);

  if (forward && !backward){
    if (now - lastStepMs >= stepDelayMs){ stepOnce(+1); lastStepMs = now; }
  } else if (backward && !forward){
    if (now - lastStepMs >= stepDelayMs){ stepOnce(-1); lastStepMs = now; }
  }

  // MQ sampling non-blocking using EMA
  if (now - lastMqMs >= MQ_INTERVAL_MS){
    lastMqMs = now;
    int r1 = analogRead(MQ1_PIN);
    int r2 = analogRead(MQ2_PIN);
    mq1Avg = (1.0f - MQ_ALPHA) * mq1Avg + MQ_ALPHA * (float)r1;
    mq2Avg = (1.0f - MQ_ALPHA) * mq2Avg + MQ_ALPHA * (float)r2;
    float mqCombined = (mq1Avg + mq2Avg) * 0.5f;
    Serial.print("MQ1 EMA: "); Serial.print(mq1Avg); Serial.print(" MQ2 EMA: "); Serial.print(mq2Avg); Serial.print(" COMBINED: "); Serial.println(mqCombined);
    if (mqCombined > MQ_THRESHOLD){
      if (!fanActive){ fanActive = true; fanServo.attach(SERVO_PIN); fanLastToggleMs = now; fanPos = false; Serial.println("Ventilación ON"); }
    } else {
      if (fanActive){ fanActive = false; fanServo.detach(); Serial.println("Ventilación OFF"); }
    }

    // read DHT once per MQ interval (non-blocking aside from library internals)
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) Serial.println("DHT read failed");
    else { Serial.print("DHT T:"); Serial.print(t); Serial.print(" H:"); Serial.println(h); }
  }

  // INA219 periodic reading (every INA_INTERVAL_MS)
  if (now - lastInaMs >= INA_INTERVAL_MS){
    lastInaMs = now;
    if (inaInitialized){
      inaBusV = ina219.getBusVoltage_V();
      inaCurrent_mA = ina219.getCurrent_mA();
      inaPower_mW = ina219.getPower_mW();
      // accumulate energy (milliwatt * hours). dt in hours = dt_ms / (1000*3600)
      if (relayState2){ // only accumulate when motor power is enabled
        double dt_h = (double)INA_INTERVAL_MS / (1000.0 * 3600.0);
        energy_mWh += (double)inaPower_mW * dt_h;
      }
      // Auto-print the INA219 readings so they are constant/automatic
      Serial.print("INA V="); Serial.print(inaBusV); Serial.print("V ");
      Serial.print("I="); Serial.print(inaCurrent_mA); Serial.print("mA ");
      Serial.print("P="); Serial.print(inaPower_mW); Serial.print("mW ");
      Serial.print("E="); Serial.print(energy_mWh); Serial.println("mWh");
    }
  }

  // non-blocking fan servo movement
  if (fanActive){
    if (now - fanLastToggleMs >= FAN_TOGGLE_MS){
      fanLastToggleMs = now;
      fanPos = !fanPos;
      if (fanPos) fanServo.write(0);
      else fanServo.write(180);
    }
  }

  // Serial commands
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd == "1"){
      relayState1 = !relayState1;
      applyRelay(0, relayState1);
      Serial.print("Relay1 state -> "); Serial.println(relayState1?"ON":"OFF");
    } else if (cmd == "2"){
      relayState2 = !relayState2;
      applyRelay(1, relayState2);
      Serial.print("Relay2 state -> "); Serial.println(relayState2?"ON":"OFF");
    } else if (cmd == "P1"){
      Serial.println("Pulse test Relay1 (300ms)");
      pulseRelayTest(0, 300);
    } else if (cmd == "P2"){
      Serial.println("Pulse test Relay2 (300ms)");
      pulseRelayTest(1, 300);
    } else if (cmd == "L"){
      relayActiveLow = !relayActiveLow;
      // reapply current states
      applyRelay(0, relayState1);
      applyRelay(1, relayState2);
      Serial.print("Relay activeLow = "); Serial.println(relayActiveLow?"YES":"NO");
    } else if (cmd.startsWith("R")){
      int v = cmd.substring(1).toInt(); if (v>0) stepDelayMs = max(1, v); Serial.print("stepDelayMs = "); Serial.println(stepDelayMs);
    } else if (cmd == "status"){
      Serial.print("Relay1:"); Serial.print(digitalRead(RELAY1_PIN)); Serial.print(" Relay2:"); Serial.print(digitalRead(RELAY2_PIN));
      Serial.print(" MQ1:"); Serial.print(mq1Avg); Serial.print(" MQ2:"); Serial.print(mq2Avg);
      Serial.print(" fan:"); Serial.println(fanActive);
      Serial.print(" V:"); Serial.print(inaBusV); Serial.print("V I:"); Serial.print(inaCurrent_mA); Serial.print("mA P:"); Serial.print(inaPower_mW); Serial.print("mW E:"); Serial.print(energy_mWh); Serial.println("mWh");
    }
  }
}