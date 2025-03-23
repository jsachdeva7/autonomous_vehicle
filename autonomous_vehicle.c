#include "autonomous_vehicle.h"

int main(void) {
  PIDController* steering_pid = NULL;  // Declare as NULL initially
  PyObject* yolo_inference;
  init(&steering_pid, &yolo_inference);

  TrafficLightBuffer tl_buffer = {0, 0, 0, "", 0, 0, 0};

  // main loop
  while (wbu_driver_step() != -1) {
    static int i = 0;
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      const double* gps_coords = wb_gps_get_values(gps);
      const unsigned char * camera_data = wb_camera_get_image(camera);
      double new_steering_angle = stay_in_lane_angle(camera_data);
      set_steering_angle(((new_steering_angle != UNKNOWN) ? new_steering_angle : 0), steering_pid);
      
      if (gps_coords) {
        check_buffer_timers(&tl_buffer);
        check_for_signal(gps_coords[0], gps_coords[1], yolo_inference, &tl_buffer);

        if (strcmp(tl_buffer.decision, "red") == 0) {
          set_speed(0.0);  // Stop the car
        } else if (strcmp(tl_buffer.decision, "yellow") == 0) {
          set_speed(30.0);  // Normal speed (simplification)
        } else if (strcmp(tl_buffer.decision, "green") == 0) {
          set_speed(30.0);  // Normal speed
        }
      } else {
          printf("GPS data not available yet.\n");
      }
    }

    ++i;
  }

  cleanup(&steering_pid, &yolo_inference);
  return 0;  
}