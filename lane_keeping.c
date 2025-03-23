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