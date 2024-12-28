// webots imports
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/display.h>


// c imports
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

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

// display
WbDeviceTag main_display;

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
const unsigned char yellow[] = {95, 187, 203};
const unsigned char lane_color[] = {156, 156, 156};
int white = 0xFFFFFF;

void init() {
  wbu_driver_init();
  
  // Initialize display
  main_display = wb_robot_get_device("main_display_new");
  if (main_display == 0) {
    printf("Error: main_display not found\n");
    // exit(1);
  }
  
  // Initialize camera properties
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);

  wb_display_attach_camera(main_display, camera);
  
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

double stay_in_lane_angle(const unsigned char *camera_image) {
  int lane_pixels = 0;
  int sum_x_lane = 0;
  int yellow_pixels = 0;
  int sum_x_yellow = 0;

  int magenta = 0xFF00FF;
  int cyan = 0xFFFF00;

  const unsigned char *pixel_data = camera_image;

  // Only look at bottom third of image
  int start_y = (camera_height * 2) / 3;  // bottom third instead of half

  // First pass: find yellow line position
  for (int y = start_y; y < camera_height; y++) {
    for (int x = 0; x < camera_width; x++) {
      int i = y * camera_width + x;
      if (is_yellow(&pixel_data[i * 4])) {
        yellow_pixels++;
        sum_x_yellow += x;
        wb_display_set_color(main_display, magenta);
        wb_display_draw_pixel(main_display, x, y);
      }
    }
  }

  if (yellow_pixels == 0)
    return UNKNOWN;

  // Calculate average yellow line position
  double yellow_avg_x = (double)sum_x_yellow / yellow_pixels;

  // Second pass: only count lane markings to the right of average yellow position
  for (int y = start_y; y < camera_height; y++) {
    for (int x = 0; x < camera_width; x++) {
      int i = y * camera_width + x;
      if (x > yellow_avg_x && is_lane_color(&pixel_data[i * 4])) {
        lane_pixels++;
        wb_display_set_color(main_display, cyan);
        wb_display_draw_pixel(main_display, x, y);
        sum_x_lane += x;
      }
    }
  }

  if (lane_pixels == 0)
    return UNKNOWN;

  // Calculate average x position for lane
  double lane_avg_x = (double)sum_x_lane / lane_pixels / camera_width;
  double yellow_avg_x_normalized = yellow_avg_x / camera_width;
  
  // Target position should be closer to the lane marking than the yellow line
  double target_x = (0.3 * yellow_avg_x_normalized + 0.7 * lane_avg_x);
  
  // Convert to angle
  return (target_x - 0.5) * camera_fov;
}

bool is_lane_color(const unsigned char* pixel) {
    bool bool_array[3] = {false, false, false};
    for (int i = 0; i < 3; ++i) {
        if (pixel[i] - lane_color[i] > 0 && pixel[i] - lane_color[i] < 50) {
            bool_array[i] = true;
        }
    }
    return bool_array[0] && bool_array[1] && bool_array[2];
}

bool is_yellow(const unsigned char* pixel) {
  int color_diff = abs(pixel[0] - yellow[0]) + abs(pixel[1] - yellow[1]) + abs(pixel[2] - yellow[2]);
  return color_diff < 30;
}

void reset_display() {
  wb_display_set_color(main_display, white);
  wb_display_fill_rectangle(main_display, 0, 0, wb_display_get_width(main_display), wb_display_get_height(main_display));
}

int main(int argc, char **argv) {
  init();

  set_speed(40.0);
  // set_steering_angle(-0.05);

  // main loop
  while (wbu_driver_step() != -1) {
    static int i = 0;

    const unsigned char * camera_data = wb_camera_get_image(camera);
    double new_steering_angle = stay_in_lane_angle(camera_data);
    if (new_steering_angle != UNKNOWN) {
      set_steering_angle(new_steering_angle);
    } else {
      set_steering_angle(0.0);
    }
    printf("new_steering_angle: %f\n", new_steering_angle);
    
    // updates sensors only every TIME_STEP milliseconds
    // if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {}
    
    reset_display();
    ++i;
  }

  wbu_driver_cleanup();

  return 0;  // ignored
}