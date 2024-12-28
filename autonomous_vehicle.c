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
int camera_width;
int camera_height;
double camera_fov;

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

// color constants
int yellow[] = {231, 219, 125};
int lane_color[] = {70, 71, 74};

void init() {
  wbu_driver_init();
  
  // Initialize camera properties
  camera = wb_robot_get_device("camera"); // Get camera device first
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);
  
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

double stay_in_lane_angle(const unsigned char *camera_data) {
  int num_pixels = camera_height * camera_width;
  const unsigned char *pixel_data = camera_data;
  int lane_pixels = 0;
  int sum_x_lane = 0;
  int yellow_pixels = 0;
  int sum_x_yellow = 0;

  for (int i = 0; i < num_pixels; ++i) {
    if (is_yellow(pixel_data)) {
      yellow_pixels++;
      sum_x_yellow += i % camera_width;
    } else if (is_lane_color(pixel_data)) {
      lane_pixels++;
      sum_x_lane += i % camera_width;
    }
  }

  if (lane_pixels == 0 || yellow_pixels == 0)
    return UNKNOWN;

  // Calculate average x position for both yellow line and lane
  double yellow_avg_x = (double)sum_x_yellow / yellow_pixels / camera_width;
  double lane_avg_x = (double)sum_x_lane / lane_pixels / camera_width;
  
  // Target position is halfway between yellow line and lane
  double target_x = (yellow_avg_x + lane_avg_x) / 2;
  
  // Convert to angle
  return (target_x - 0.5) * camera_fov;
}

bool is_lane_color(int* pixel) {
  bool[] bool_array = {false, false, false};
  for (int i = 0; i < 3; ++i) {
    if (pixel[i] - lane_color[i] > 0 && pixel[i] - lane_color[i] < 50) {
      bool_array[i] = true;
    }
  }
  return bool_array[0] && bool_array[1] && bool_array[2];
}

bool is_yellow(int* pixel) {
  int color_diff = abs(pixel[0] - yellow[0]) + abs(pixel[1] - yellow[1]) + abs(pixel[2] - yellow[2]);
  return color_diff < 30;
}

int main(int argc, char **argv) {
  init();

  set_speed(10.0);
  set_steering_angle(-0.05);

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
