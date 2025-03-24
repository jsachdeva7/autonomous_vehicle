#ifndef TRAFFIC_DETECTION_H
#define TRAFFIC_DETECTION_H

#include <Python.h>

#include <webots/camera.h>

#include "constants.h"
#include "devices.h"

typedef struct {
    int red_count;
    int green_count;
    int yellow_count;
    char decision[10];
    int red_timer;
    int green_timer;
    int yellow_timer;
} TrafficLightBuffer;

void check_for_signal(double x, double y, PyObject *pModule, TrafficLightBuffer* tl_buffer);
void check_buffer_timers(TrafficLightBuffer* tl_buffer);


#endif // TRAFFIC_DETECTION_H