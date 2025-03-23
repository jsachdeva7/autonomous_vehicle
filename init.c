#include "init.h"

void init(PIDController** steering_pid, int time_step) {
    wbu_driver_init();
    wbu_driver_set_dipped_beams(true);
    wbu_driver_set_antifog_lights(true);
        
    // Initialize display
    main_display = wb_robot_get_device("main_display_new");
    if (main_display == 0) {
      printf("Error: main_display not found\n");
      // exit(1);
    }
    
    // Initialize camera properties
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, time_step);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);

    // Initialize GPS
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, time_step);
  
    // Allocate memory for PID controller
    *steering_pid = malloc(sizeof(PIDController));
    if (*steering_pid == NULL) {
        printf("Error: Failed to allocate memory for PID controller\n");
        exit(1);  // Handle memory allocation failure
    }
    
    // Set coefficients for PID controller
    (*steering_pid)->kp = 16.0f;
    (*steering_pid)->ki = 0.0f;
    (*steering_pid)->kd = 0.5f;
    (*steering_pid)->T = 0.05f;
    (*steering_pid)->tau = 0.02f;
    (*steering_pid)->limMin = -0.5f;
    (*steering_pid)->limMax = 0.5f;

    // Reset controller
    pid_init(*steering_pid);

    // Set initial speed and angle
    set_speed(30.0);
    wbu_driver_set_steering_angle(0.0);
}