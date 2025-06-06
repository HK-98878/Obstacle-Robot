#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <Arduino.h>
#include <Servo.h>

/*
QTR and Line Following Code:
- Sensor Setup
- Line follow controller
  (QTR read is local to the .cpp file)
*/

void setupLineSensors();
int lineFollow(int16_t *left, int16_t *right);

// Flags used to signal characteristics of the detected line
#define LINE_FLAGS_FULL_WIDTH 0b00000001
#define LINE_FLAGS_SPLIT      0b00000010
#define LINE_FLAGS_NO_LINE    0b00000100
void readLine(float *h_offset, int8_t *l_width, uint8_t *flags);

#endif