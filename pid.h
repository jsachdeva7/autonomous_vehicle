#ifndef PID_H
#define PID_H

/**
 * @file pid.c
 * @brief PID Controller Implementation
 * 
 * This file defines the PIDController struct and function prototypes for implementing
 * a PID controller.
 * The controller, in general, helps regulate a system by minimizing the difference
 * between a desired set point and a measured value.
 * In this project, the PID controller is used to minimize the difference between the 
 * desired steering angle and the measured steering angle of the BMW X5.
 */

/**
 * @struct PIDController
 * @brief Structure representing a PID controller
 *
 * This struct holds all necessary parameters for a PID controller, 
 * including gains, filtering parameters, limits, and memory 
 * for past values used in integral and derivative calculations.
 */
typedef struct {
    float kp; /**< Proportional gain */
    float ki; /**< Integral gain */
    float kd; /**< Derivative gain */

    float tau; /**< Derivative low-pass filter constant */

    float limMin; /**< Minimum output limit */
    float limMax; /**< Maximum output limit */

    float T; /**< Sample time in seconds */

    float integrator; /**< Integral term accumulator */
    float differentiator; /**< Defferentiated error value */
    float prevError; /**< previous error for derivative caclulation */
    float prevMeasurement; /**< Previous measurement to compute derivative */

    float out; /**< PID controller output */
} PIDController;

/**
 * @brief Initializes the PID controller
 * 
 * This function resets the internal states of the PID controller,
 * such as the integrator, differentiator, previous error, and output.
 * 
 * @param pid Pointer to the PIDController structure to be initialized.
 */
void pid_init(PIDController *pid);

/**
 * @brief Updates the PID controller with a new measurement.
 * 
 * Computes the control output using the proportional, integral, and
 * derivative terms, including anti-windup and filtering.
 * 
 * @param pid Pointer to the PIDController structure. 
 * @param setpoint The desired target value.
 * @param measurement The current measured value.
 * @return float The computed control output.
 */
float pid_update(PIDController *pid, float setpoint, float measurement);

#endif // PID_H