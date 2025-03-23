#ifndef AUTONOMOUS_VEHICLES_H
#define AUTONOMOUS_VEHICLES_H
#define BUFFER_SIZE 7

#include <stdbool.h>

typedef struct {
    int red_count;
    int green_count;
    int yellow_count;
    char decision[10];
    int red_timer;
    int green_timer;
    int yellow_timer;
} TrafficLightBuffer;

// Function prototypes
// double stay_in_lane_angle(const unsigned char *camera_data);
void check_for_signal(double x, double y, PyObject* pModule, TrafficLightBuffer* buffer);

#endif /* AUTONOMOUS_VEHICLES_H */