#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>
#include <Motoron.h>
#include <Servo.h>

/*
Motor Control Code:
- General motor setup
- Set speeds (left/right)
- Kill switch handling

- Complex obstacle processes - lava pit and stair climb
*/

void setupKillSwitch();
void handleKillSwitch(bool *kill_switch_active);

void setupMotors(bool *kill_switch_active);
void stopAllMotors();

void setLRSpeeds(int16_t left, int16_t right, bool invert_front);

void setLineSensorServo(bool extended);

int lavapit_cross(unsigned long start_time, int16_t *left, int16_t *right);
int climbStairs(unsigned long start_time, int16_t *left,int16_t *right, bool *invert_front);
int zipwire_extend_arms(unsigned long start_time, int16_t *left, int16_t *right);

void treadmill();

#endif
