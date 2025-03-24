#include "../include/control.h"

void set_steering_angle(double desired_angle, PIDController* steering_pid) {
    double steering_angle = wbu_driver_get_steering_angle();
    double steering_adjustment = pid_update(steering_pid, desired_angle, steering_angle);
    
    // Apply the adjustment, ensure within max change per step
    if (steering_adjustment > 0.1) {
      steering_adjustment = 0.1;
    } else if (steering_adjustment < -0.1) {
        steering_adjustment = -0.1;
    }
  
    // Update steering angle w/ PID correction
    steering_angle += steering_adjustment;
  
    // Clamp steering angle to vehicle limits
    if (steering_angle > 0.5) {
      steering_angle = 0.5;
    } else if (steering_angle < -0.5) {
      steering_angle = -0.5;
    }
  
    // Apply steering angle
    wbu_driver_set_steering_angle(steering_angle);
}

void set_speed(double desired_speed) {
    desired_speed = (desired_speed > 150) ? 50 : desired_speed;
    wbu_driver_set_cruising_speed(desired_speed);
}