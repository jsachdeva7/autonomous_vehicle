#ifndef PID_H
#define PID_H

#include <Python.h>

#include <webots/camera.h>

#include "constants.h"
#include "autonomous_vehicle.h"
#include "devices.h"

void check_for_signal(double x, double y, PyObject *pModule, TrafficLightBuffer* tl_buffer);
void check_buffer_timers(TrafficLightBuffer* tl_buffer);


#endif // PID_H