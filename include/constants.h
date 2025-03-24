#ifndef CONSTANTS_H
#define CONSTANTS_H

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Traffic detection intervals (ms)
#define GREEN_DETECTION_INTERVAL 5000
#define YELLOW_DETECTION_INTERVAL 5000
#define RED_DETECTION_INTERVAL 3000

// Hex constants
#define WHITE   0xFFFFFF
#define MAGENTA 0xFF00FF
#define CYAN    0xFFFF00
#define RED     0xFF6557

// RGB constants
static const unsigned char yellow[] = {109, 194, 208};
static const unsigned char lane_color[] = {190, 190, 190};

#endif /* CONSTANTS_H */