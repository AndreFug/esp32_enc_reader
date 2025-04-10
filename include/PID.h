#ifndef PID_H
#define PID_H


class PID {
public:
    
    PID(float _kp, float _ki, float _kd);
    float control(float target, float current, float dt);
    void set_constraints(float min, float max);
    void set_windup(float _windup);
    
private:
    float kp = 0;
    float ki = 0;
    float kd = 0;

    float last_integral = 0;
    float windup = 0;
    float last_error = 0;

    float max_output = 0;
    float min_output = 0;
};

#endif