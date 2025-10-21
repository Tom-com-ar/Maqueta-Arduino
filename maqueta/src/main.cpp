/*
  Maqueta control extendida
  - 4 relés (IN1=D10, IN2=D11, IN3=D12, IN4=D13)
  - Stepper 28BYJ-48 con ULN2003 (D4..D7)
  - 2 pulsadores: D2 = forward, D3 = back
  - Servo en D9 (controlado por el relé 4)
  - DHT22 en D8
  - MQ135 sensores en A0 y A1
  - INA219 medición de corriente
*/

#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// --- Pines ---
const int RELAY1_PIN = 10; // luminaria A
const int RELAY2_PIN = 11; // luminaria B
const int RELAY3_PIN = 12; // motor paso a paso
const int RELAY4_PIN = 13; // servo control

const int BUTTON_FWD_PIN = 2;
const int BUTTON_BWD_PIN = 3;

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
bool servoActive = false;
int servoPos = 0;
bool servoUp = true;
unsigned long lastServoMs = 0;
const unsigned long SERVO_DELAY_MS = 10;

// --- Relés ---
bool relayActiveLow = true;
bool relayState1 = false;
bool relayState2 = false;
bool relayState3 = false;
bool relayState4 = false;

// --- Función para aplicar estado a los relés ---
void applyRelay(int idx, bool on) {
  int pin;
  if (idx == 0) pin = RELAY1_PIN;
  else if (idx == 1) pin = RELAY2_PIN;
  else if (idx == 2) pin = RELAY3_PIN;
  else if (idx == 3) pin = RELAY4_PIN;
  else return;

  if (relayActiveLow) digitalWrite(pin, on ? LOW : HIGH);
  else digitalWrite(pin, on ? HIGH : LOW);
}

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
unsigned long stepDelayMs = 2;

// --- Botones ---
unsigned long lastDebounceF = 0, lastDebounceB = 0;
const unsigned long DEBOUNCE_MS = 30;
int lastReadF = HIGH, lastReadB = HIGH;

// --- MQ ---
float mq1Avg = 0.0f, mq2Avg = 0.0f;
const float MQ_ALPHA = 0.25f;
unsigned long lastMqMs = 0;
const unsigned long MQ_INTERVAL_MS = 1000;

// --- INA219 ---
Adafruit_INA219 ina219;
unsigned long lastInaMs = 0;
const unsigned long INA_INTERVAL_MS = 1000;
bool inaInitialized = false;

// --- Funciones auxiliares ---
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
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  // Inicializar relés apagados
  applyRelay(0, relayState1);
  applyRelay(1, relayState2);
  applyRelay(2, relayState3);
  applyRelay(3, relayState4);

  pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BWD_PIN, INPUT_PULLUP);

  pinMode(STEPPER_PIN1, OUTPUT);
  pinMode(STEPPER_PIN2, OUTPUT);
  pinMode(STEPPER_PIN3, OUTPUT);
  pinMode(STEPPER_PIN4, OUTPUT);
  writeStepper(0);

  pinMode(MQ1_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);

  dht.begin();

  if (!ina219.begin()) {
    Serial.println("INA219 no encontrado");
  } else {
    Serial.println("INA219 inicializado");
    inaInitialized = true;
  }

  fanServo.attach(SERVO_PIN);
  fanServo.write(90);
  fanServo.detach();

  mq1Avg = analogRead(MQ1_PIN);
  mq2Avg = analogRead(MQ2_PIN);
  lastMqMs = millis();
}

// --- Loop ---
void loop(){
  unsigned long now = millis();

  // Lectura de botones con antirrebote
  int rf = digitalRead(BUTTON_FWD_PIN);
  int rb = digitalRead(BUTTON_BWD_PIN);
  if (rf != lastReadF) { lastDebounceF = now; lastReadF = rf; }
  if (rb != lastReadB) { lastDebounceB = now; lastReadB = rb; }

  bool forward = false, backward = false;
  if ((now - lastDebounceF) > DEBOUNCE_MS) forward = (rf == LOW);
  if ((now - lastDebounceB) > DEBOUNCE_MS) backward = (rb == LOW);

  // Motor paso a paso solo si el relé 3 está activo
  if (relayState3) {
    if (forward && !backward) {
      if (now - lastStepMs >= stepDelayMs) {
        stepOnce(+1);
        lastStepMs = now;
      }
    } else if (backward && !forward) {
      if (now - lastStepMs >= stepDelayMs) {
        stepOnce(-1);
        lastStepMs = now;
      }
    }
  }

  // Movimiento del servo si relé 4 está activo
  if (relayState4) {
    if (!servoActive) {
      fanServo.attach(SERVO_PIN);
      servoActive = true;
      Serial.println("Servo activado (movimiento continuo)");
    }

    if (now - lastServoMs >= SERVO_DELAY_MS) {
      lastServoMs = now;
      if (servoUp) servoPos++;
      else servoPos--;
      fanServo.write(servoPos);
      if (servoPos >= 180) servoUp = false;
      if (servoPos <= 0) servoUp = true;
    }

  } else if (servoActive) {
    fanServo.detach();
    servoActive = false;
    Serial.println("Servo detenido");
  }

  // --- Lectura MQ ---
  if (now - lastMqMs >= MQ_INTERVAL_MS){
    lastMqMs = now;
    int r1 = analogRead(MQ1_PIN);
    int r2 = analogRead(MQ2_PIN);
    mq1Avg = (1.0f - MQ_ALPHA) * mq1Avg + MQ_ALPHA * (float)r1;
    mq2Avg = (1.0f - MQ_ALPHA) * mq2Avg + MQ_ALPHA * (float)r2;
    Serial.print("MQ1: "); Serial.print(mq1Avg);
    Serial.print(" MQ2: "); Serial.println(mq2Avg);

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      Serial.print("DHT T: "); Serial.print(t);
      Serial.print(" H: "); Serial.println(h);
    }
  }

  // --- Lectura INA219 ---
  if (now - lastInaMs >= INA_INTERVAL_MS){
    lastInaMs = now;
    if (inaInitialized){
      float V = ina219.getBusVoltage_V();
      float I = ina219.getCurrent_mA();
      float P = ina219.getPower_mW();
      Serial.print("V="); Serial.print(V);
      Serial.print("V I="); Serial.print(I);
      Serial.print("mA P="); Serial.print(P);
      Serial.println("mW");
    }
  }

  // --- Comandos por Serial ---
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "1"){
      relayState1 = !relayState1;
      applyRelay(0, relayState1);
      Serial.print("Relay1 -> "); Serial.println(relayState1 ? "ON" : "OFF");
    } 
    else if (cmd == "2"){
      relayState2 = !relayState2;
      applyRelay(1, relayState2);
      Serial.print("Relay2 -> "); Serial.println(relayState2 ? "ON" : "OFF");
    } 
    else if (cmd == "3"){
      relayState3 = !relayState3;
      applyRelay(2, relayState3);
      Serial.print("Relay3 -> "); Serial.println(relayState3 ? "ON" : "OFF");
    } 
    else if (cmd == "4"){
      relayState4 = !relayState4;
      applyRelay(3, relayState4);
      Serial.print("Relay4 (Servo) -> "); Serial.println(relayState4 ? "ON" : "OFF");
    }
  }
}
