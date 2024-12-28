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


int main(int argc, char **argv) {
  wbu_driver_init();

  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

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
