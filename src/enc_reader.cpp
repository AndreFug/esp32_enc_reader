#include "enc_reader.h"
#include <Arduino.h>

// === ENCODER 1 ===
static const byte ENCODER1_PIN_A = 32;
static const byte ENCODER1_PIN_B = 33;
volatile long encoder1Count = 0;
volatile bool encoder1Direction = true;
byte encoder1PinALast = LOW;

// === ENCODER 2 ===
static const byte ENCODER2_PIN_A = 19;
static const byte ENCODER2_PIN_B = 21;
volatile long encoder2Count = 0;
volatile bool encoder2Direction = true;
byte encoder2PinALast = LOW;

// For RPM calculation
static const float CPR = 64.0;
long encoder1CountPrev = 0;
long encoder2CountPrev = 0;
float speed1_rpm = 0;
float speed2_rpm = 0;

void IRAM_ATTR encoder1ISR() {
  int Lstate = digitalRead(ENCODER1_PIN_A);
  if ((encoder1PinALast == LOW) && (Lstate == HIGH)) {
    if (digitalRead(ENCODER1_PIN_B) == LOW && encoder1Direction) {
      encoder1Direction = false;
    } else if (digitalRead(ENCODER1_PIN_B) == HIGH && !encoder1Direction) {
      encoder1Direction = true;
    }
  }
  encoder1PinALast = Lstate;
  encoder1Count += encoder1Direction ? -1 : 1;
}

void IRAM_ATTR encoder2ISR() {
  int Lstate = digitalRead(ENCODER2_PIN_A);
  if ((encoder2PinALast == LOW) && (Lstate == HIGH)) {
    if (digitalRead(ENCODER2_PIN_B) == LOW && encoder2Direction) {
      encoder2Direction = false;
    } else if (digitalRead(ENCODER2_PIN_B) == HIGH && !encoder2Direction) {
      encoder2Direction = true;
    }
  }
  encoder2PinALast = Lstate;
  encoder2Count += encoder2Direction ? -1 : 1;
}

void setupEncoders() {
  pinMode(ENCODER1_PIN_A, INPUT);
  pinMode(ENCODER1_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1ISR, CHANGE);

  pinMode(ENCODER2_PIN_A, INPUT);
  pinMode(ENCODER2_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2ISR, CHANGE);
}
  
void updateEncoderStats() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  if (dt < 0.01) return; // avoid divide by zero

  lastUpdate = now;

  long count1 = encoder1Count;
  long count2 = encoder2Count;

  long delta1 = count1 - encoder1CountPrev;
  long delta2 = count2 - encoder2CountPrev;

  encoder1CountPrev = count1;
  encoder2CountPrev = count2;

  float speed1_pulsesPerSec = delta1 / dt;
  float speed2_pulsesPerSec = delta2 / dt;

  speed1_rpm = (speed1_pulsesPerSec / CPR) * 60.0;
  speed2_rpm = (speed2_pulsesPerSec / CPR) * 60.0;
}
float getSpeed1RPM() { return speed1_rpm; }
float getSpeed2RPM() { return speed2_rpm; }


float getAverageRPM() {
  return fabs((speed1_rpm + speed2_rpm) / 2.0);
}

float getRPMDifference() {
  return fabs(speed1_rpm - speed2_rpm);
}

bool getEncoder1Direction() {
  return encoder1Direction;
}

bool getEncoder2Direction() {
  return encoder2Direction;
}

long getEncoder1Count() {
  return encoder1Count;
}

long getEncoder2Count() {
  return encoder2Count;
}
