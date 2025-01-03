#ifndef PID_H
#define PID_H

typedef struct {
    // Controller gains
    float kp;
    float ki;
    float kd;

    // derivative low-pass filter constant
    float tau;

    // output limits
    float limMin;
    float limMax;

    // sample time (in seconds)
    float T;

    // controller "memory"
    float integrator;
    float differentiator;
    float prevError;
    float prevMeasurement;

    // Controller output
    float out;
} PIDController;

void pid_init(PIDController *pid);
float pid_update(PIDController *pid, float setpoint, float measurement);

#endif