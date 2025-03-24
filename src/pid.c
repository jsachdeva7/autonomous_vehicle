/**
 * @file pid.c
 * @brief PID Controller Implementation
 * 
 * This file contains the implementation of a generic PID controller.
 */

#include "../include/pid.h"

void pid_init(PIDController *pid) {
    // Clear controller variables
    pid->integrator = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevError = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float pid_update(PIDController *pid, float setpoint, float measurement) {
    // Compute error signal
    float error = setpoint - measurement;

    // Proportional term
    float proportional = pid->kp * error;

    // Integral term (trapezoidal integration)
    pid->integrator = pid->integrator + 0.5f * pid->ki * pid->T * (error + pid->prevError);

    // Anti-windup: dynamic integrator clamping
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

    // Clamp integrator within allowed limits
    if (pid->integrator > limMaxInt) {
        pid->integrator = limMaxInt;
    } else if (pid->integrator < limMinInt) {
        pid->integrator = limMinInt;
    }

    // Derivative term (band-limited differentiator)
    pid->differentiator = (2.0f * pid->kd * (measurement - pid->prevMeasurement)
                           + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    // Compute final output and enforce limits
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // Store error and measurement for next iteration
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return computed output
    return pid->out;
}