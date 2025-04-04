#include <Arduino.h>
#include "enc_reader.h"

/*
TODO
add encoder feedback for each state
create / tune U_turn right/left

*/

// === Motor Control Pins ===
const int INA1 = 12;
const int INA2 = 13;
const int INB1 = 14;
const int INB2 = 27;

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

// === Function to Control Motors ===
void setMotorState(State state) {
  switch (state) {
    case STOP:
      digitalWrite(INA1, LOW); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, LOW);
      break;

    case FORWARD:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW);
      break;

    case REVERSE:
      digitalWrite(INA1, LOW); digitalWrite(INA2, HIGH);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      break;

    case TURN_RIGHT:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      break;

    case TURN_LEFT:
      digitalWrite(INA1, LOW); digitalWrite(INA2, HIGH);
      digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW);
      break;

    case U_TURN:
      digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
      delay(500);
      break;

    case U_TURN_RIGHT:
      NULL;
      break;

    case U_TURN_LEFT:
      NULL;
      break;
  }
}

void setup() {
  Serial.begin(115200);
  setupEncoders();

  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  setMotorState(STOP);
}

void loop() {
  updateEncoderStats();

  // === Serial Input Parser ===
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // remove whitespace

    if (input == "STOP") currentState = STOP;
    else if (input == "FORWARD") currentState = FORWARD;
    else if (input == "REVERSE") currentState = REVERSE;
    else if (input == "RIGHT") currentState = TURN_RIGHT;
    else if (input == "LEFT") currentState = TURN_LEFT;
    else if (input == "U_TURN") currentState = U_TURN;
    else if (input == "U_TURN_RIGHT") currentState = U_TURN_RIGHT;
    else if (input == "U_TURN_LEFT") currentState = U_TURN_LEFT;

    Serial.print("Updated state to: ");
    Serial.println(input);
  }

  // === Act on Current State ===
  setMotorState(currentState);
}
