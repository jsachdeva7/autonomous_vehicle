#ifndef INIT_H
#define INIT_H

#include <stdio.h>
#include <stdlib.h>
#include <Python.h>

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>

#include "devices.h"
#include "pid.h"
#include "control.h"

// #define TIME_STEP 50
// #define UNKNOWN 99999.99

void init(PIDController** steering_pid, int time_step, PyObject** yolo_inference);
PyObject* initialize_python();

#endif /* INIT_H */