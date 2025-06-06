#ifndef DISTSENSOR_H
#define DISTSENSOR_H

/*
Distance Sensor and Wall Following Code:
- Sensor read functions
- Wall following controllers
*/

#include <Arduino.h>
#include "globals.h"

// Distance sensor pins and parameters (statics to avoid exposure to other files which include this one)
#define FRONT_DIST_PIN A8
#define WALL_DIST_PIN A9

const static float K = 53;
const static float alpha = 1.18;
const static float Vref = 3.3;
#define ADC_RESOLUTION (12)

// Reading buffer for moving median
static float readings[NUM_DIST_READINGS];
static int bufIndex = 0;

void setupDistSensor();

int wall_turn_right(unsigned long start_time, int16_t *left, int16_t *right); // Returns 1 once action is complete
void wall_follow_PID(float setpoint, int16_t *left, int16_t *right);

// Templated functions used to avoid issues with Arduino's type inconsistency between the A8 and A9 pins
template<typename PinType>
float readDistance(PinType pin) {
  uint32_t raw = analogRead(pin);
  float V = raw * Vref / 4095.0;
  return K * pow(V, -alpha);
}

template<typename PinType>
float median_filter(PinType pin) {

  // Get new reading
  readings[bufIndex] = readDistance(pin);
  bufIndex = (bufIndex + 1) % NUM_DIST_READINGS;

  // Copy and sort readings to find median
  float sorted[NUM_DIST_READINGS];
  memcpy(sorted, readings, sizeof(readings));

  // Simple bubble sort (since NUM_DIST_READINGS is small)
  for (int i = 0; i < NUM_DIST_READINGS - 1; i++) {
    for (int j = i + 1; j < NUM_DIST_READINGS; j++) {
      if (sorted[j] < sorted[i]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  return sorted[NUM_DIST_READINGS / 2];  // Return median
}

#endif
