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
#include <stdlib.h>
#include <stddef.h>


// custom file imports
#include "pid.h"
#include "autonomous_vehicles.h"
#include "helper.h"

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

// PID Controller
PIDController *steering_pid = NULL;  // Declare as NULL initially

void init() {
  printf("init() happening...\n");
  fflush(stdout);
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

  // wb_display_attach_camera(main_display, camera);
  
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

  // Initialize PID controller
  steering_pid = malloc(sizeof(PIDController));
  if (steering_pid == NULL) {
    printf("Error: Failed to allocate memory for PID controller\n");
    exit(1);  // Handle memory allocation failure
  }
  
  steering_pid->kp = 20.0f;
  steering_pid->ki = 0.0f;
  steering_pid->kd = 0.0f;
  steering_pid->T = 0.05f;
  steering_pid->tau = 0.02f;
  steering_pid->limMin = -0.5f;
  steering_pid->limMax = 0.5f;
}

void set_steering_angle(double desired_angle) {
  double steering_adjustment = pid_update(steering_pid, desired_angle, steering_angle);
  
  // Apply the adjustment, ensure within max change per step
  if (steering_adjustment > 0.1) {
    steering_adjustment = 0.1;
  } else if (steering_adjustment < -0.1) {
      steering_adjustment = -0.1;
  }

  // Update steering angle w/ PID correction
  steering_angle += steering_adjustment;

  // Clamp steering angle to vehicle limits
  if (steering_angle > 0.5) {
    steering_angle = 0.5;
  } else if (steering_angle < -0.5) {
    steering_angle = -0.5;
  }

  // Apply steering angle
  wbu_driver_set_steering_angle(steering_angle);
}

void set_speed(double desired_speed) {
  desired_speed = (desired_speed > 150) ? 50 : desired_speed;
  wbu_driver_set_cruising_speed(desired_speed);
}

double stay_in_lane_angle(const unsigned char *camera_image) {
  // First, create a new image from the camera data
  WbImageRef camera_frame = wb_display_image_new(main_display, camera_width, camera_height, camera_image, WB_IMAGE_BGRA);
  
  // Paste the camera frame to the display
  wb_display_image_paste(main_display, camera_frame, 0, 0, false);
  
  // Free the image reference since we're done with it
  wb_display_image_delete(main_display, camera_frame);
  
  // Now continue with your existing pixel drawing code
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
  
  // Draw yellow line indicator
  int vanishing_x = camera_width / 2;
  wb_display_set_color(main_display, magenta);
  wb_display_draw_line(main_display, 
                      (int)yellow_avg_x, camera_height,        // bottom point
                      vanishing_x, start_y);                   // top point at vanishing point

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
  double lane_avg_x = (double)sum_x_lane / lane_pixels;
  
  // Draw white lane indicator
  wb_display_set_color(main_display, cyan);
  wb_display_draw_line(main_display, 
                      (int)lane_avg_x, camera_height,          // bottom point
                      vanishing_x, start_y);                   // top point at vanishing point

  // Continue with your existing calculations
  lane_avg_x = lane_avg_x / camera_width;
  double yellow_avg_x_normalized = yellow_avg_x / camera_width;
  
  // Target position should be closer to the lane marking than the yellow line
  double target_x = (0.4 * yellow_avg_x_normalized + 0.6 * lane_avg_x);
  
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

int main(void) {
  print_hello();
  init();
  pid_init(steering_pid);
  set_speed(30.0);

  // main loop
  while (wbu_driver_step() != -1) {
    static int i = 0;
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      const unsigned char * camera_data = wb_camera_get_image(camera);
      double new_steering_angle = stay_in_lane_angle(camera_data);
      if (new_steering_angle != UNKNOWN) {
        set_steering_angle(new_steering_angle);
      }
      // printf("new_steering_angle: %f\n", new_steering_angle);
    }

    ++i;
  }

  wbu_driver_cleanup();
  free(steering_pid);

  return 0;  
}