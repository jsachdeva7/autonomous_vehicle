#include "lane_keeping.h"

bool is_yellow(const unsigned char* pixel) {
    int color_diff = abs(pixel[0] - yellow[0]) + abs(pixel[1] - yellow[1]) + abs(pixel[2] - yellow[2]);
    return color_diff < 30;
}

bool is_valid_yellow(const unsigned char* pixel, int x, int y, const unsigned char* image) {
    if (!is_yellow(pixel)) return false;

    // Count how many yellow pixels are nearby horizontally
    int yellow_width = 0;
    for (int dx = -5; dx <= 5; dx++) { // Check ±5 pixels in X direction
        int nx = x + dx;
        if (nx >= 0 && nx < camera_width && is_yellow(&image[(y * camera_width + nx) * 4])) {
          yellow_width++;
        }
    }
  
    return yellow_width < 6;  // Ignore wide patches (crosswalks)    
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

bool is_valid_lane_color(const unsigned char* pixel, int x, int y, const unsigned char* image) {
    if (!is_lane_color(pixel)) return false;

    // Count how many "lane" pixels are nearby horizontally
    int lane_pixel_width = 0;
    for (int dx = -5; dx <=5; dx++) {
      int nx = x + dx;
      if (nx >= 0 && nx < camera_width && is_yellow(&image[(y * camera_width + nx) * 4])) {
        lane_pixel_width++;
      }
    }
  
    return lane_pixel_width < 6;
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
  
    const unsigned char *pixel_data = camera_image;
  
    // Only look at bottom third of image
    int start_y = (camera_height * 2) / 3;  // bottom third instead of half
  
    // Draw part of camera scanned over:
    int height_from_top = camera_height / 4;
    int height_from_bot = camera_height / 8;
    wb_display_set_color(main_display, RED);
    wb_display_draw_line(main_display,
                        0, camera_height - height_from_bot,
                        camera_width / 2, height_from_top);
    wb_display_draw_line(main_display,
                        camera_width, camera_height - height_from_bot,
                        camera_width / 2, height_from_top);
    // wb_display_draw_line(main_display, 
    // (int)lane_avg_x, camera_height,          // bottom point
    // vanishing_x, start_y); // top point
  
    // First pass: find yellow line position
    for (int y = start_y; y < camera_height; y++) {
      for (int x = 0; x < camera_width; x++) {
        int i = y * camera_width + x;
        if (is_valid_yellow(&pixel_data[i * 4], x, y, pixel_data)) {
          yellow_pixels++;
          sum_x_yellow += x;
          wb_display_set_color(main_display, MAGENTA);
          wb_display_draw_pixel(main_display, x, y);
        }
      }
    }
  
    // Calculate average yellow line position
    double yellow_avg_x = (yellow_pixels > 0) ? (double)sum_x_yellow / yellow_pixels : -1;
    
    // Draw yellow line indicator if detected
    int vanishing_x = camera_width / 2;
    if (yellow_avg_x != -1) {
      wb_display_set_color(main_display, MAGENTA);
      wb_display_draw_line(main_display, 
                          (int)yellow_avg_x, camera_height,        // bottom point
                          vanishing_x, start_y);                   // top point at vanishing point
    }
  
    // Second pass: only count lane markings to the right of average yellow position
    for (int y = start_y; y < camera_height; y++) {
      for (int x = 0; x < camera_width; x++) {
        int i = y * camera_width + x;
        if ((yellow_avg_x == -1 || x > yellow_avg_x) && is_valid_lane_color(&pixel_data[i * 4], x, y, pixel_data)) {
          lane_pixels++;
          wb_display_set_color(main_display, CYAN);
          wb_display_draw_pixel(main_display, x, y);
          sum_x_lane += x;
        }
      }
    }
  
    // Calculate average x position for lane
    double lane_avg_x = (lane_pixels > 0) ? (double)sum_x_lane / lane_pixels : -1;
    
    // Draw white lane indicator if detected
    if (lane_avg_x != -1) {
      wb_display_set_color(main_display, CYAN);
      wb_display_draw_line(main_display, 
                          (int)lane_avg_x, camera_height,          // bottom point
                          vanishing_x, start_y);                   // top point at vanishing point
    }
    
    // Handle missing lane lines
    double target_x;
    double correction_factor = -1.2;
    
    if (yellow_avg_x == -1 && lane_avg_x == -1) {
      return UNKNOWN;  // No lane lines detected
    } else if (yellow_avg_x == -1) {
      target_x = correction_factor * lane_avg_x / camera_width;  // Use only the white lane
    } else if (lane_avg_x == -1) {
      if (yellow_pixels < 10) {
        return UNKNOWN;
      }
      target_x = correction_factor * yellow_avg_x / camera_width;  // Use only the yellow lane
    } else {
      target_x = (0.4 * (yellow_avg_x / camera_width) + 0.6 * (lane_avg_x / camera_width));
    }
  
    // Convert to angle
    return (target_x - 0.5) * camera_fov;
}