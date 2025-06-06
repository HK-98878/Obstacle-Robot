#ifndef GLOBALS_H
#define GLOBALS_H

/*
Key Variables:
- Obstacle sequence for general control
- PID parameters and thresholds for the wall and line following
- Obstacle speeds
- Line sequence for the line following
*/

#include <Arduino.h>
#include <Servo.h>

// Add this line to enable serial prints
//   (at least the ones wrapped in #ifdef statements or the CONDITIONAL_SERIAL_PRINT[LN] macros)
// #define SERIAL_LOGGING

// Add this line to enable printing of the wall distance sensor
//   (when disabled - for alignnent)
//#define DISTANCE_DEBUG

// Add this line to enable manual calibration of the line sensor
// #define MANUAL_LINE_CALIBRATION

// Add this line to disable the spring arm setup stage
//   (i.e. they will not be locked in the closed position)
// #define NO_SPRING_ARMS

// Add this line to disable the WiFi kill-switch
//   Used heavily during testing and debugging, since it significantly speeds up compile times
//   and the switch is not needed at that point
#define NO_WIFI

// Macro function which will expand to serial prints if SERIAL_LOGGING is defined, and are empty lines otherwise
//   Empty while is used to avoid issues with certain cases - e.g. `if(<condition>) CONDITIONAL_SERIAL_PRINT(...);`
//   This would lead to errors if the macro was empty
#ifdef SERIAL_LOGGING
#define CONDITIONAL_SERIAL_PRINT(msg) Serial.print(msg)
#define CONDITIONAL_SERIAL_PRINTLN(msg) Serial.println(msg)
#else
#define CONDITIONAL_SERIAL_PRINT(msg) do {} while (0)
#define CONDITIONAL_SERIAL_PRINTLN(msg) do {} while (0)
#endif


#pragma region General Setup
// ------------- Event Sequencing -------------
typedef enum CurrentAction {
  ACTION_NONE,

  ACTION_LINE_FOLLOW,
  ACTION_DEPLOY_SENSOR,
  ACTION_RETRACT_SENSOR,

  ACTION_WALL_FOLLOW,
  ACTION_WALL_FOLLOW_RAMP,
  ACTION_WALL_FOLLOW_TURN,

  ACTION_STAIR_APPROACH,
  ACTION_STAIR_CLIMB,

  ACTION_LAVA_PIT_APPROACH,
  ACTION_LAVA_PIT_CROSS,

  ACTION_ZIPWIRE_RUNUP,
  ACTION_ZIPWIRE_EXTEND,
  ACTION_ZIPWIRE_DRIVE,
} CurrentAction;


// Section 1
/*
#define OBSTACLE_COUNT 1
const CurrentAction obstacleSequence[OBSTACLE_COUNT] = { ACTION_DEPLOY_SENSOR, ACTION_LINE_FOLLOW, ACTION_RETRACT_SENSOR };
*/


// Section 2
/*
#define OBSTACLE_COUNT 9
const CurrentAction obstacleSequence[OBSTACLE_COUNT] = {
  ACTION_WALL_FOLLOW_RAMP,  // ┬ Ramp
  ACTION_WALL_FOLLOW_TURN,  // ┘
  ACTION_STAIR_APPROACH,    // ┬ Stairs
  ACTION_STAIR_CLIMB,       // │
  ACTION_STAIR_CLIMB,       // │
  ACTION_STAIR_CLIMB,       // ┘
  ACTION_WALL_FOLLOW,       // ┬ Exit stairs
  ACTION_WALL_FOLLOW_TURN,  // ┘
  ACTION_WALL_FOLLOW,       // ┬ Causeway
  ACTION_WALL_FOLLOW_TURN,  // ┘
  ACTION_WALL_FOLLOW        // ─ Lunar, up to end
};
*/

// Section 3

#define OBSTACLE_COUNT 7
const CurrentAction obstacleSequence[OBSTACLE_COUNT] = {
  ACTION_WALL_FOLLOW_RAMP,   // ┬ Up ramp, find line
  ACTION_WALL_FOLLOW_TURN,   // │
  ACTION_DEPLOY_SENSOR,      // ┘
  ACTION_LINE_FOLLOW,        // ─ Ramp, two corners
  ACTION_LAVA_PIT_APPROACH,  // ─ Lava pit
  ACTION_DEPLOY_SENSOR,      // ┬ Follows a line until detecting a cliff, then triggers zipline sequence
  ACTION_ZIPWIRE_RUNUP       // ┘
};

// --------------------------------------------
#pragma endregion


#pragma region Obstacles
// ----------------- Lava Pit -----------------
#define LAVA_PIT_APPROACH_SPEED 800
#define LAVA_PIT_CROSS_SPEED 800

#define ARM_SERVO_UP_ANGLE 0
#define ARM_SERVO_DOWN_ANGLE_R 90
#define ARM_SERVO_DOWN_ANGLE_F 95

#define CLIFF_THRESHOLD 180

#define LAVA_PIT_SERVO_MOVE_DELAY 800
#define LAVA_PIT_CROSS_DURATION 1500

// ------------------ Stairs ------------------
#define STAIR_APPROACH_SPEED 500
#define STAIR_SPEED 800

#define STAIR_COUNT 3
#define TALL_STAIR 2

#define STAIR_SERVO_DELAY_DURATION 1000
#define STAIR_STEP_DURATION 50
#define STAIR_DRIVE_DURATION 500

// --------------------------------------------

// ----------------- Zip-line -----------------
#define ZIPWIRE_SERVO_DELAY 1000
#define ZIPWIRE_DRIVE_SPEED 600

#define ZIPWIRE_DRIVE_CLIFF_SPEED 500
#define ZIPWIRE_DRIVE_CLIFF_DELAY 500

// --------------------------------------------

#pragma endregion


#pragma region Wall Following
// -------- Distance Sensor Parameters --------

#define NUM_DIST_READINGS 5  // Size of the median filter window
// Note that this filter is *shared* between sensors

const float NORMAL_FRONT_DIST = 98.0;

// -------------- Wall Following --------------

#define WALL_DES_DIST 120
#define WALL_DES_DIST_RAMP 60

const float Kp_Wall = 7;
const float Kd_Wall = 9;

const int16_t wallFollowMaxSpeed = 800, wallFollowMinSpeed = 0, wallFollowBaseSpeed = 500;
const int16_t wallTurnReverseSpeed = 400;
const int16_t wallTurnRightSpeed = 700;

#define WALL_TURN_REVERSE_TIME 150
#define WALL_TURN_REVERSE_TIME_2 150
#define WALL_TURN_DELAY_TIME 300
#define WALL_TURN_TURN_TIME 1500

// --------------------------------------------
#pragma endregion


#pragma region Line Following
// -------------- Line Following --------------
const float Kp_Line = 70;
const float Kd_Line = -4;

#define LINE_TURN_THRESHOLD 5

const int16_t lineFollowBaseSpeed = 500, lineFollowMaxSpeed = 800, lineFollowMinSpeed = 400, lineFollowTurnSpeed = 600;

// ----------- Line Follow Sequence -----------

typedef enum NextLineEvent {
  EndOfLine,

  ThickLine,

  YJunction_Left,
  YJunction_Right,
  YJunction_Straight,

  SharpCorner_Left, // 90 degree turns where reversing is needed to avoid hitting the wall in the turn motion
  SharpCorner_Right
} NextLineEvent;

// Section 1
/*
#define LINE_EVENT_COUNT 3
const NextLineEvent lineEventSequence[LINE_EVENT_COUNT] = { ThickLine, SharpCorner_Right, EndOfLine}; //, SharpCorner_Right, EndOfLine 
*/

// Section 3

#define LINE_EVENT_COUNT 3
const NextLineEvent lineEventSequence[LINE_EVENT_COUNT] = { 
  SharpCorner_Right, // Around corner
  EndOfLine,         // Start of lava pit
  EndOfLine          // Safety on gap cross (although we're already prob over the edge)
};


// --------------------------------------------
#pragma endregion

#endif