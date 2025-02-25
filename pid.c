#include "pid.h"
#include <stdio.h>

void pid_init(PIDController *pid) {
    printf("PID initializing...\n");
    fflush(stdout);
    // clear controller variables
    pid->integrator = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevError = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
    printf("PID initialized!\n");
    fflush(stdout);
}

float pid_update(PIDController *pid, float setpoint, float measurement) {
    // error signal
    float error = setpoint - measurement;

    // proportional term
    float proportional = pid->kp * error;

    // integral term
    pid->integrator = pid->integrator + 0.5f * pid->ki * pid->T * (error + pid->prevError);

    // anti-windup via dynamic integrator clamping
    float limMinInt, limMaxInt;
    if (pid->limMax > proportional) {
        limMaxInt = pid->limMax - proportional;
    } else {
        limMaxInt = 0.0f;
    }

    if (pid->limMin < proportional) {
        limMinInt = pid->limMin - proportional;
    } else {
        limMinInt = 0.0f;
    }

    // clamp integrator
    if (pid->integrator > limMaxInt) {
        pid->integrator = limMaxInt;
    } else if (pid->integrator < limMinInt) {
        pid->integrator = limMinInt;
    }

    // derivative (band-limited differentiator)
    pid->differentiator = (2.0f * pid->kd * (measurement - pid->prevMeasurement)
                           + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    // compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // store error and measurement for next time
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // return controller output
    return pid->out;
}