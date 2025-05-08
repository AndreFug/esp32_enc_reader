#include <Arduino.h>
#include "enc_reader.h"
#include "PID.h"
#define fwd true
#define rev false

// === Motor Control Pins ===
const int INA1 = 12;
const int INA2 = 13;
const int INB1 = 14;
const int INB2 = 27;
const int PWMA = 2;
const int PWMB = 4;
const int Enable = 22;

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

// PIDDDDD
PID pid1(3000.0, 50000.0, 0.0);
PID pid2(3000.0, 50000.0, 0.0);
bool current_dirA = false;
bool current_dirB = false;

void set_motor_dir_1(bool direction) {
  if (direction == fwd) {
    digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW); // Forward
  } else {
    digitalWrite(INA1, LOW); digitalWrite(INA2, HIGH);
  }
}

void set_motor_dir_2(bool direction) {
  if (direction == fwd) {
    digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW); // Forward
  } else {
    digitalWrite(INB1, LOW); digitalWrite(INB2, HIGH);
  }
}


void setMotor1Speed(int speed) {
  if (speed < 0 && current_dirA==fwd) {
    set_motor_dir_1(rev); // Reverse
    current_dirA = rev;
  } else if (speed > 0 && current_dirA==rev) {
    set_motor_dir_1(fwd);
    current_dirA = fwd;
  }
  ledcWrite(CH_A, abs(speed));
}

void setMotor2Speed(int speed) {
  if (speed < 0 && current_dirB==fwd) {
    set_motor_dir_2(rev); // Reverse
    current_dirB = rev;
  } else if (speed > 0 && current_dirB==rev) {
    set_motor_dir_2(fwd);
    current_dirB = fwd;
  }
  ledcWrite(CH_B, abs(speed));
}

void reset_motor_driver() {
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  ledcWrite(CH_A, 0);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  ledcWrite(CH_B, 0);
  delay(10);
  current_dirA = true;
  current_dirB = true;
  set_motor_dir_1(true);
  set_motor_dir_2(true);
}

void setMotorState(State state) {
  switch (state) {
    case STOP:
      digitalWrite(INA1, LOW); digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW); digitalWrite(INB2, LOW);
      ledcWrite(CH_A, 0);
      ledcWrite(CH_B, 0);
      break;

      case FORWARD: {
        // Set motor directions for forward movement.
        digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW);
        digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW);
      }


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

void printMotorAState() {
  Serial.print("current_dirA=");
  Serial.print(current_dirA ? "FWD" : "REV");
  Serial.print(" | INA1=");
  Serial.print(digitalRead(INA1));
  Serial.print(" INA2=");
  Serial.print(digitalRead(INA2));
  Serial.print(" | Act PWM:");
  Serial.println(ledcRead(CH_A));
}

void printMotorBState() {
  Serial.print("current_dirB=");
  Serial.print(current_dirB ? "FWD" : "REV");
  Serial.print(" | INB1=");
  Serial.print(digitalRead(INB1));
  Serial.print(" INB2=");
  Serial.print(digitalRead(INB2));
  Serial.print(" | Act PWM:");
  Serial.println(ledcRead(CH_B));
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


  pid1.set_constraints(-255, 255);
  pid2.set_constraints(-255, 255);
  pid1.set_windup(255);
  pid2.set_windup(255);

  digitalWrite(INA1, HIGH); digitalWrite(INA2, LOW); // Forward
  digitalWrite(INB1, HIGH); digitalWrite(INB2, LOW); // Forward
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, HIGH);
}


unsigned long t_last = micros();
float target_speed_A = 0.00;
float target_speed_B = 0.00;
float freq = 100;
float t_step = 1e6/freq;

void loop() {
  
  unsigned long dt = micros() - t_last;

  if (dt > t_step) { 
    int bytes = Serial.available(); // Check for available serial data. 

    
    if (bytes >= 6) {
      // Process serial data. 
      char buffer[6];
      Serial.readBytes(buffer, 6);
      Serial.print("Received bytes: ");
      for (int i = 0; i < 6; i++) {
        Serial.print((int)buffer[i]);
        Serial.print(" ");
      }
      Serial.println();
      int id1 = (int) buffer[0];
      int dir1 = (int) buffer[1];
      float spd1 = (int) buffer[2];
      int id2 = (int) buffer[3];
      int dir2 = (int) buffer[4];
      float spd2 = (int) buffer[5];

      if (id1 == 0x0a) {
        reset_motor_driver();
      } else {
        if (dir1 == 1) target_speed_A = spd1*1e-3;
        else target_speed_A = -spd1*1e-3;
        
        if (dir2 == 1) target_speed_B = spd2*1e-3;
        else target_speed_B = -spd2*1e-3;
      }
    }

    // Stop interrupts to avoid race condition. -> Cannot read encoder values
    noInterrupts();  updateEncoderStats(); interrupts(); // Turn interrupts back on -> can read encoder values
    float current_speed_A = -get_speed_1();
    float current_speed_B = get_speed_2();
    
    int output_A = (int) pid1.control(target_speed_A, current_speed_A, dt*1e-6);
    int output_B = (int) pid2.control(target_speed_B, current_speed_B, dt*1e-6);
    setMotor1Speed(output_A);
    setMotor2Speed(output_B);

    // printMotorAState();
    // printMotorBState();
    t_last += t_step; // Advance time by step.
  }}