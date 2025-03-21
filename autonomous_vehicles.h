#ifndef AUTONOMOUS_VEHICLES_H
#define AUTONOMOUS_VEHICLES_H

#include <stdbool.h>
// Function prototypes
bool is_yellow(const unsigned char* pixel);
bool is_lane_color(const unsigned char* pixel);
bool is_valid_lane_color(const unsigned char* pixel, int x, int y, const unsigned char* image);
bool is_valid_yellow(const unsigned char* pixel, int x, int y, const unsigned char* image);
double stay_in_lane_angle(const unsigned char *camera_data);
void set_speed(double desired_speed);
void set_steering_angle(double desired_angle);
void check_for_signal(double x, double y);
void init();
int initialize_python();

#endif /* AUTONOMOUS_VEHICLES_H */