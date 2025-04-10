#include "enc_reader.h"
#include <Arduino.h>
unsigned long lastUpdate = 0;

// === ENCODER 1 ===
static const byte ENCODER1_PIN_A = 32;
static const byte ENCODER1_PIN_B = 33;
volatile long encoder1Count = 0;
volatile bool encoder1Direction = true;
volatile byte encoder1PinALast = LOW;


// === ENCODER 2 ===
static const byte ENCODER2_PIN_A = 19;
static const byte ENCODER2_PIN_B = 21;
volatile long encoder2Count = 0;
volatile bool encoder2Direction = true;
volatile byte encoder2PinALast = LOW;

// For RPM calculation
static const float CPR = 64.0;
long encoder1CountPrev = 0;
long encoder2CountPrev = 0;
float speed1_rpm = 0;
float speed2_rpm = 0;
float speed_1 = 0;
float speed_2 = 0;

float gear_ratio = 0.0075;  // 1/50 * 9/24
float meters_per_output_rev = 0.1 * 3.1415;
float meters_per_motor_rev = meters_per_output_rev * gear_ratio; // 
float meters_per_count = meters_per_motor_rev / CPR;

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
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  if (dt < 0.00001) return; // avoid divide by zero

  lastUpdate = now;

  speed_1 = encoder1Count / dt;
  speed_2 = encoder2Count / dt;

  encoder1Count = 0;
  encoder2Count = 0;
  // speed1_rpm = -(speed1_pulsesPerSec / CPR) * 60.0;
  // speed2_rpm = (speed2_pulsesPerSec / CPR) * 60.0;
}
// float getSpeed1RPM() { return speed1_rpm; }
// float getSpeed2RPM() { return speed2_rpm; }

float get_speed_1() {return speed_1 * meters_per_count; }
float get_speed_2() {return speed_2 * meters_per_count; }

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
