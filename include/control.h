#ifndef CONTROL_H
#define CONTROL_H
#include <webots/vehicle/driver.h>
#include "pid.h"

void set_speed(double desired_speed);
void set_steering_angle(double desired_angle, PIDController* steering_pid);

#endif /* CONTROL_H */