#include "LineFollow.h"
#include "DistSensor.h"
#include "Motors.h"

#ifndef NO_WIFI
#include "WiFiComm.h"

// -------------- Wi-Fi  Details --------------
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
const unsigned long WIFI_TIMEOUT_MS = 4000;

// --------------------------------------------
#endif

// Start disabled, and await button press
bool kill_switch_active = true;

// Current action tracking and start time for timed actions
CurrentAction current_action = ACTION_NONE;
uint8_t current_action_ptr = 0;
unsigned long action_start_time = 0;

void setup() {
#ifdef SERIAL_LOGGING
  Serial.begin(9600);
  while (!Serial && millis() < 1500)
    ;
#endif
  delay(500);

  setupKillSwitch();
#ifndef NO_WIFI
  setupWiFi(ssid, password, WIFI_TIMEOUT_MS);
#endif
  setupMotors(&kill_switch_active);
  setupDistSensor();

  kill_switch_active = true;
  stopAllMotors();
  delay(500);

  CONDITIONAL_SERIAL_PRINT("Awaiting Button ... ");
  while (kill_switch_active) handleKillSwitch(&kill_switch_active);
  CONDITIONAL_SERIAL_PRINTLN("Starting Calibration");

  setLineSensorServo(true);
  delay(500);
  setupLineSensors();
  setLineSensorServo(false);

  CONDITIONAL_SERIAL_PRINTLN("Setup Fininshed");

  kill_switch_active = true;
  stopAllMotors();

  current_action = obstacleSequence[current_action_ptr];

  delay(1000);
}


void loop() {
  handleKillSwitch(&kill_switch_active);
#ifndef NO_WIFI
  handleWiFiCommands(&kill_switch_active);
#endif

  // If kill switch is enabled, or the obstacle sequence is over, exit the function
  if (kill_switch_active || current_action_ptr >= OBSTACLE_COUNT) {
    // CONDITIONAL_SERIAL_PRINTLN(readDistance(WALL_DIST_PIN) - WALL_DES_DIST);
    stopAllMotors();
    return;
  }

  static int16_t left = 400, right = 400;  // Starting speeds
  static bool invert_front_wheels = false;
  float dist;

  switch (current_action) {
    case ACTION_NONE:  // No defined actions - do not move
      left = right = 0;
      break;

    case ACTION_DEPLOY_SENSOR:
      setLineSensorServo(true);
      delay(500);
      current_action = obstacleSequence[++current_action_ptr];
      action_start_time = millis();
      break;
    case ACTION_RETRACT_SENSOR:
      setLineSensorServo(false);
      delay(500);
      current_action = obstacleSequence[++current_action_ptr];
      action_start_time = millis();
      break;
    case ACTION_LINE_FOLLOW:            // Follow a defined line (see globals.h)
      if (lineFollow(&left, &right)) {  // End obstacle if returning 1 (hit all defined line events)
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      dist = readDistance(FRONT_DIST_PIN);
      if (dist > CLIFF_THRESHOLD) {
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;

    case ACTION_WALL_FOLLOW:  // Follow a straight left-side wall
      wall_follow_PID(WALL_DES_DIST, &left, &right);

      dist = readDistance(FRONT_DIST_PIN);
      if (dist < (NORMAL_FRONT_DIST - 20)) {  // Go to next action
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;
    case ACTION_WALL_FOLLOW_RAMP:
      wall_follow_PID(WALL_DES_DIST_RAMP, &left, &right);
      dist = readDistance(FRONT_DIST_PIN);
      static int ramp_state = 0;
      if (dist < NORMAL_FRONT_DIST - 20 && ramp_state == 0) ramp_state = 1;       // Start of the ramp
      else if (dist > NORMAL_FRONT_DIST + 60 && ramp_state == 1) ramp_state = 2;  // End of the ramp
      else if (dist < NORMAL_FRONT_DIST - 20 && ramp_state == 2) {
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;
    case ACTION_WALL_FOLLOW_TURN:                               // Make a right-hand turn from one wall to the next
      if (wall_turn_right(action_start_time, &left, &right)) {  // Go to next action (generally will be back to ACTION_WALL_FOLLOW, but might be a line)
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;

    case ACTION_STAIR_APPROACH:
      left = right = STAIR_APPROACH_SPEED;
      dist = readDistance(FRONT_DIST_PIN);
      if (dist < (NORMAL_FRONT_DIST - 15)) {
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;
    case ACTION_STAIR_CLIMB:
      if (climbStairs(action_start_time, &left, &right, &invert_front_wheels)) {
        action_start_time = millis();
        current_action = obstacleSequence[++current_action_ptr];  // Start time is reset so climb can be sequenced three times in a row
                                                                  //   (after ACTION_STAIR_APPROACH for safety)
      }
      break;

    case ACTION_LAVA_PIT_APPROACH:
      left = right = LAVA_PIT_APPROACH_SPEED;
      dist = readDistance(FRONT_DIST_PIN);

      if (dist > CLIFF_THRESHOLD) {  // If the reading is over the thresold (seen cliff), start the cross sequence
        current_action = ACTION_LAVA_PIT_CROSS;
        action_start_time = millis();
        setLineSensorServo(false);
      }
      break;
    case ACTION_LAVA_PIT_CROSS:
      if (lavapit_cross(action_start_time, &left, &right)) {
        current_action = obstacleSequence[++current_action_ptr];
      }
      break;

    case ACTION_ZIPWIRE_RUNUP:
      lineFollow(&left, &right);  // Line follow for alignment
      dist = readDistance(FRONT_DIST_PIN);

      if (dist > CLIFF_THRESHOLD) {  // If the reading is over the thresold (seen cliff), start the zipline deploy sequence
        current_action = ACTION_ZIPWIRE_EXTEND;
        action_start_time = millis();
        setLineSensorServo(false);
      }
      break;
    case ACTION_ZIPWIRE_EXTEND:  // Deploy the spring arms
      if (zipwire_extend_arms(action_start_time, &left, &right)) {
        action_start_time = millis();
        current_action = ACTION_ZIPWIRE_DRIVE;
      }
      break;
    case ACTION_ZIPWIRE_DRIVE:  // Drive forward permanently once the arms are deployed - adjust this
      left = right = ZIPWIRE_DRIVE_SPEED;
      break;
  }
  setLRSpeeds(left, right, invert_front_wheels);
}
