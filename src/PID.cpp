#include "PID.h"
#include "Arduino.h"

PID::PID(float _kp, float _ki, float _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
};

float PID::control(float target, float current, float dt) {

    float e = target - current;

    float P = e * kp;
    float I = e * dt * ki + last_integral;
    float D = (e-last_error) * kd / dt;

    // Windup control
    I = constrain(I, -windup, windup);
    
    last_integral = I;
    last_error = e;

    return constrain(P + I + D, min_output, max_output);
};

void PID::set_constraints(float _min, float _max) {
    min_output = _min;
    max_output = _max;
}

void PID::set_windup(float _windup){
    windup = _windup;
}