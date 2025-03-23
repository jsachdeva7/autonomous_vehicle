// Webots imports
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>

// C Library imports
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

// Python import
#include <Python.h>

// Header file imports
#include "constants.h"
#include "init.h"
#include "pid.h"
#include "autonomous_vehicle.h"
#include "devices.h"
#include "control.h"
#include "lane_keeping.h"

// PID Controller
PIDController* steering_pid = NULL;  // Declare as NULL initially

void check_for_signal(double x, double y, PyObject *pModule, TrafficLightBuffer *buffer) {
  // Skip detection if the decision is already made as green
  if (strcmp(buffer->decision, "") != 0) {
    return;  // Skip detection if a decision is committed to
    printf("Decision made: %s. No need for checking at the moment.\n", buffer->decision);
  } 

  bool within_y = y > -75 && y < -60;
  bool within_x = x < 48.5 && x > 46.5;
  if (within_x && within_y) {
    const unsigned char *image = wb_camera_get_image(camera);
    if (!image) {
      printf("Could not capture image.\n");
      return;
    }

    PyObject *pFunc = PyObject_GetAttrString(pModule, "detect_traffic_light");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        PyErr_Print();
        printf("Error: Failed to load function detect_traffic_light\n");
        Py_XDECREF(pFunc);
        return;
    }

    // Convert raw image data to Python bytes object
    PyObject *pBytes = PyBytes_FromStringAndSize((const char *)image, camera_width * camera_height * 4);
    if (!pBytes) {
      PyErr_Print();
      printf("Error: Failed to create Python bytes object\n");
      Py_DECREF(pFunc);
      return;
    }

    PyObject *pWidth = PyLong_FromLong(camera_width);
    PyObject *pHeight = PyLong_FromLong(camera_height);

    // Call detect_traffic_light("traffic_light.jpg")
    PyObject *pArgs = PyTuple_Pack(3, pBytes, pWidth, pHeight);
    printf("YOLO Running.\n");
    PyObject *pValue = PyObject_CallObject(pFunc, pArgs);

    Py_DECREF(pBytes);
    Py_DECREF(pWidth);
    Py_DECREF(pHeight);
    Py_DECREF(pArgs);
    Py_DECREF(pFunc);

    if (pValue) {
      const char *raw_result = PyUnicode_AsUTF8(pValue);
      char raw_result_copy[100];

      // Copy the raw_result into raw_result_copy (mutable string)
      strncpy(raw_result_copy, raw_result, sizeof(raw_result_copy) - 1);
      raw_result_copy[sizeof(raw_result_copy) - 1] = '\0'; // Null-terminate the string

      // Now use strtok with the mutable copy
      char *result = strtok(raw_result_copy, ",");
      Py_DECREF(pValue);

      printf("Detected Traffic Light: %s\n", result);

      // Increment the counts and timers based on the result
      if (strcmp(result, "red") == 0) {
        buffer->red_count++;
        printf("Incremented red_count to %d\n", buffer->red_count);
        if (buffer->red_count >= 2) {
          printf("Decision: STOP. Detectionless for 3 seconds.\n");
          strcpy(buffer->decision, "red");
          buffer->red_count = 0;
          buffer->yellow_count = 0;
          buffer->green_count = 0;
        }
      } else if (strcmp(result, "green") == 0) {
        buffer->green_count++;
        printf("Incremented green_count to %d\n", buffer->green_count);
        if (buffer->green_count >= 2) {
          printf("Decision: GO. Detectionless for 5 seconds.\n");
          strcpy(buffer->decision, "green");
          buffer->red_count = 0;
          buffer->yellow_count = 0;
          buffer->green_count = 0;
        }
      } else if (strcmp(result, "yellow") == 0) {
        buffer->yellow_count++;
        printf("Incremented yellow_count to %d\n", buffer->yellow_count);
        if (buffer->yellow_count >= 2) {
          printf("Decision: CAUTION. Detectionless for 5 seconds\n");
          strcpy(buffer->decision, "green");
          buffer->red_count = 0;
          buffer->yellow_count = 0;
          buffer->green_count = 0;
        }
      }
    }
  }
}

int main(void) {
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
        if (tl_buffer.green_timer >= 5000 || tl_buffer.red_timer >= 3000 || tl_buffer.yellow_timer >= 5000) { // 5 seconds, 3 seconds, 5 seconds
          strcpy(tl_buffer.decision, "");  // Reset decision
          tl_buffer.green_timer = 0;  // Reset green timer
          tl_buffer.red_timer = 0;  // Reset red timer
          tl_buffer.yellow_timer = 0;  // Reset yellow timer
          tl_buffer.green_count = 0; // Reset counts 
          tl_buffer.red_count = 0;
          tl_buffer.yellow_count = 0;
        } else {
          if (strcmp(tl_buffer.decision, "green") == 0) {
            tl_buffer.green_timer += TIME_STEP;
          } else if (strcmp(tl_buffer.decision, "red") == 0) {
            tl_buffer.red_timer += TIME_STEP;
          } else if (strcmp(tl_buffer.decision, "yellow") == 0) {
            tl_buffer.yellow_timer += TIME_STEP;
          }
        }
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

  wbu_driver_cleanup();
  free(steering_pid);
  if (yolo_inference) {  // Ensure yolo_inference is valid before freeing
    Py_DECREF(yolo_inference);
  }
  Py_Finalize();

  return 0;  
}