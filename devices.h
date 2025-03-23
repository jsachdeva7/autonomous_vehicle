#ifndef DEVICES_H
#define DEVICES_H

#include <stddef.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/lidar.h>

// Camera
extern WbDeviceTag camera;
extern int camera_width;
extern int camera_height;
extern double camera_fov;

// Display
extern WbDeviceTag main_display;

// SICK laser
extern WbDeviceTag sick;
extern int sick_width;
extern double sick_range;
extern double sick_fov;

// Speedometer
extern WbDeviceTag display;
extern int display_width;
extern int display_height;
extern WbImageRef speedometer_image;

// GPS
extern WbDeviceTag gps;
extern double gps_coords[3];
extern double gps_speed;

#endif // DEVICES_H