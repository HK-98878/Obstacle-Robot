// DistSensor.cpp
#include "DistSensor.h"


void setupDistSensor() {
  analogReadResolution(ADC_RESOLUTION);

  // Initialize readings
  for (int i = 0; i < NUM_DIST_READINGS; i++) {
    readings[i] = readDistance(WALL_DIST_PIN);
  }
}

int wall_turn_right(unsigned long start_time, int16_t *left, int16_t *right) {
  unsigned long t = millis();  // get current time
  if (t - start_time < WALL_TURN_REVERSE_TIME) {
    *left = -wallTurnReverseSpeed;
    *right = -wallTurnReverseSpeed;
    return 0;
  } else if (t - start_time < WALL_TURN_REVERSE_TIME + WALL_TURN_DELAY_TIME)
    return 0;
  else if (t - start_time < WALL_TURN_REVERSE_TIME + WALL_TURN_DELAY_TIME + WALL_TURN_TURN_TIME / 2) {
    *left = wallTurnRightSpeed;
    *right = 0;
    return 0;
  } else if (t - start_time < 2 * WALL_TURN_REVERSE_TIME + WALL_TURN_DELAY_TIME + WALL_TURN_TURN_TIME / 2) {
    *left = -wallTurnReverseSpeed;
    *right = -wallTurnReverseSpeed;
    return 0;
  } else if (t - start_time < WALL_TURN_REVERSE_TIME + WALL_TURN_REVERSE_TIME_2 + 2 * WALL_TURN_DELAY_TIME + WALL_TURN_TURN_TIME / 2)
    return 0;
  else if (t - start_time < WALL_TURN_REVERSE_TIME + WALL_TURN_REVERSE_TIME_2 + 2 * WALL_TURN_DELAY_TIME + WALL_TURN_TURN_TIME) {
    *left = wallTurnRightSpeed;
    *right = wallTurnRightSpeed/3;
    return 0;
  } else return 1;
}

void wall_follow_PID(float setpoint, int16_t *left, int16_t *right) {
  static int16_t prev_error = 0;

  float dist_smoothed = median_filter(WALL_DIST_PIN);  /// try without filter?
  float error = setpoint - dist_smoothed;         // if error neg (further from wall) turn left

  int16_t correction = Kp_Wall * error + Kd_Wall * (error - prev_error);

  *left = constrain(wallFollowBaseSpeed + correction, wallFollowMinSpeed, wallFollowMaxSpeed);  // -maxSpeed, constrain to 0 if you dont want wheel to rotate backwards
  *right = constrain(wallFollowBaseSpeed - correction, wallFollowMinSpeed, wallFollowMaxSpeed);

  prev_error = error;
}