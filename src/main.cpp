#include <Arduino.h>
#include "enc_reader.h"
#include "BluetoothSerial.h"

// === Motor Control Pins ===
const int INA1 = 12;
const int INA2 = 13;
const int INB1 = 14;
const int INB2 = 27;
const int PWMA = 2;
const int PWMB = 4;

// === PWM Channels ===
const int PWM_FREQ = 1000;      // 1 kHz
const int PWM_RES = 8;          // 8-bit resolution (0–255)
const int CH_A = 2;             // PWM channel for Motor A
const int CH_B = 4;             // PWM channel for Motor B

int motorSpeed = 255; // Default motor speed (0–255)

// === State Definitions ===
enum State {
  STOP,
  FORWARD,
  REVERSE,
  TURN_RIGHT,
  TURN_LEFT,
  U_TURN,
  U_TURN_RIGHT,
  U_TURN_LEFT
};

State currentState = STOP;
// BluetoothSerial SerialBT;

void setMotorState(State state) {
  switch (state) {
    case STOP:
      digitalWrite(INA1, LOW); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, LOW);
      ledcWrite(CH_A, 0);
      ledcWrite(CH_B, 0);
      break;

    case FORWARD:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW);
      ledcWrite(CH_A, motorSpeed);
      ledcWrite(CH_B, motorSpeed);
      break;

    case REVERSE:
      digitalWrite(INA1, LOW); digitalWrite(INA2, HIGH);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      ledcWrite(CH_A, motorSpeed);
      ledcWrite(CH_B, motorSpeed);
      break;

    case TURN_RIGHT:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      ledcWrite(CH_A, motorSpeed);
      ledcWrite(CH_B, motorSpeed);
      break;

    case TURN_LEFT:
      digitalWrite(INA1, LOW); digitalWrite(INA2, HIGH);
      digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW);
      ledcWrite(CH_A, motorSpeed);
      ledcWrite(CH_B, motorSpeed);
      break;

    case U_TURN:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      ledcWrite(CH_A, motorSpeed);
      ledcWrite(CH_B, motorSpeed);
      delay(500);
      digitalWrite(INA1, LOW); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, LOW);
      break;

    case U_TURN_RIGHT:
      // Placeholder for encoder-based tuning
      break;

    case U_TURN_LEFT:
      // Placeholder for encoder-based tuning
      break;
  }
}

void setup() {
  Serial.begin(115200);
  // SerialBT.begin("ESP32_BT", true);
  setupEncoders();

  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  // Setup PWM channels
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);

  // Attach PWM to pins
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);

  setMotorState(STOP);
}

void loop() {
  updateEncoderStats();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  
    input.trim();

    if (input == "STOP") currentState = STOP;
    else if (input == "FORWARD") currentState = FORWARD;
    else if (input == "REVERSE") currentState = REVERSE;
    else if (input == "RIGHT") currentState = TURN_RIGHT;
    else if (input == "LEFT") currentState = TURN_LEFT;
    else if (input == "U_TURN") currentState = U_TURN;
    else if (input == "U_RIGHT") currentState = U_TURN_RIGHT;
    else if (input == "U_LEFT") currentState = U_TURN_LEFT;
    else if (input.startsWith("SPEED ")) {
      int newSpeed = input.substring(6).toInt();
      motorSpeed = constrain(newSpeed, 0, 255);
      Serial.println("Speed set to " + String(motorSpeed));
    }

    Serial.print("Updated state to: ");
    Serial.println(input);
  }

  setMotorState(currentState);
}
