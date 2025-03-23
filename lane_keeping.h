#ifndef LANE_KEEPING_H
#define LANE_KEEPING_H

#include <stdbool.h>

#include "devices.h"
#include "colors.h"

bool is_yellow(const unsigned char* pixel);
bool is_valid_yellow(const unsigned char* pixel, int x, int y, const unsigned char* image);
bool is_lane_color(const unsigned char* pixel);
bool is_valid_lane_color(const unsigned char* pixel, int x, int y, const unsigned char* image);

#endif /* LANE_KEEPING_H */