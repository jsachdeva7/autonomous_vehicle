#include "init.h"

void init(PIDController** steering_pid, int time_step, PyObject** yolo_inference) {
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

    // Obtain yolo_inference function
    *yolo_inference = initialize_python();
    if (!*yolo_inference) {
        printf("Python initialization failed.\n");
    }

    // Warm up YOLO
    PyObject *pWarm_Up = PyObject_GetAttrString(*yolo_inference, "warm_up_model");
    if (!pWarm_Up || !PyCallable_Check(pWarm_Up)) {
        PyErr_Print();
        printf("Error: Failed to load function warm_up_model\n");
        Py_XDECREF(pWarm_Up);
        Py_DECREF(*yolo_inference);
        return;
    }

    // Call warm-up function
    PyObject *pWarm_Up_Result = PyObject_CallObject(pWarm_Up, NULL);
    if (!pWarm_Up_Result) {
        PyErr_Print();
        printf("Error: warm_up_model() function call failed.\n");
    } else {
        Py_DECREF(pWarm_Up_Result);
    }

    Py_DECREF(pWarm_Up);
}

PyObject* initialize_python() {
  PyStatus status;
  PyConfig config;
  PyConfig_InitPythonConfig(&config);

  // Set Python home (replace with your actual path)
  status = PyConfig_SetString(&config, &config.home, L"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312");
  if (PyStatus_Exception(status)) {
    PyConfig_Clear(&config);
    return NULL;
  }

  // Initialize Python with the modified configuration
  status = Py_InitializeFromConfig(&config);
  PyConfig_Clear(&config);

  // Run Python code
  PyRun_SimpleString("import sys; sys.stdout = sys.__stdout__");
  PyRun_SimpleString("print('Hello from Python inside C!'); sys.stdout.flush()");
  if (PyErr_Occurred()) {
    PyErr_Print();
  }

  PyObject *pModule = PyImport_ImportModule("yolo_inference");
    if (!pModule) {
      PyErr_Print();
      printf("Error: Failed to import yolo_inference.py\n");
      return NULL;
  }

  return pModule;
}