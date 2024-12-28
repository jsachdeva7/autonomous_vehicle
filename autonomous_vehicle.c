// webots imports
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>


// c imports
#include <math.h>
#include <stdio.h>
#include <string.h>

// custom file imports
#include <autonomous_vehicles.h>

// to be used as array indices
enum { X, Y, Z };


// constants
#define TIME_STEP 50
#define UNKNOWN 99999.99

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void init() {
  wbu_driver_init();
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);
}

void set_steering_angle(double desired_angle) {
  double steering_change = desired_angle - steering_angle;
  if (steering_change > 0) {
    steering_change = (steering_change > 0.1) ? 0.1 : steering_change;
  } else if (steering_change < 0) {
    steering_change = (steering_change < -0.1) ? -0.1 : steering_change;
  }
  steering_angle += steering_change;
  if (steering_angle > 0.5) {
    steering_angle = 0.5;
  } else if (steering_angle < -0.5) {
    steering_angle = -0.5;
  }
  wbu_driver_set_steering_angle(steering_angle);
}

void set_speed(double desired_speed) {
  desired_speed = (desired_speed > 30) ? 30 : desired_speed;
  wbu_driver_set_cruising_speed(desired_speed);
}

int main(int argc, char **argv) {
  init();

  set_speed(10.0);
  set_steering_angle(-0.1);

  // main loop
  while (wbu_driver_step() != -1) {
    static int i = 0;
    
    // updates sensors only every TIME_STEP milliseconds
    // if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {}
      
    ++i;
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}
