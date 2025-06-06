#include "Motors.h"
#include "globals.h"
#include <Wire.h>

Servo frontArmServo, backArmServo, LS_Servo, frontSpringServo, backSpringServo;
const int FRONT_SERVO_PIN = 29, BACK_SERVO_PIN = 31, LS_SERVO_PIN = 27, SPRING_SERVO1_PIN = 33, SPRING_SERVO2_PIN = 25;

MotoronI2C frontMotors(17), backMotors(18);

#define buttonPin 40
void setupKillSwitch() {
  pinMode(buttonPin, INPUT_PULLUP);
}
void handleKillSwitch(bool *kill_switch_active) {
  // Debouncing variables
  static int lastButtonState = HIGH;  // the previous reading from the button pin
  static int buttonState = HIGH;      // the current button state
  static unsigned long lastDebounceTime = 0;

  const unsigned long DEBOUNCE_MS = 50;

  int reading = digitalRead(buttonPin);

  // Check if the button state changed
  if (reading != lastButtonState) {
    // Reset the debounce timer
    lastDebounceTime = millis();
  }

  // If enough time has passed since the last button change
  if (millis() - lastDebounceTime > DEBOUNCE_MS && reading != buttonState) {  // AND if the button state has actually changed

    buttonState = reading;

    // Only toggle when the button is pressed DOWN (LOW for INPUT_PULLUP)
    if (buttonState == LOW) {
      if (*kill_switch_active)  // reattach servos when going from inactive to active
      {
        frontArmServo.attach(FRONT_SERVO_PIN);
        backArmServo.attach(BACK_SERVO_PIN);
        LS_Servo.attach(LS_SERVO_PIN);
      }
      *kill_switch_active = !*kill_switch_active;
    }
  }

  lastButtonState = reading;
}

void setupMotors(bool *kill_switch_active) {
  Wire1.begin();

  frontMotors.setBus(&Wire1);
  backMotors.setBus(&Wire1);

  frontMotors.reinitialize();
  frontMotors.clearResetFlag();
  frontMotors.clearMotorFaultUnconditional();

  backMotors.reinitialize();
  backMotors.clearResetFlag();
  backMotors.clearMotorFaultUnconditional();

  frontArmServo.attach(FRONT_SERVO_PIN);
  backArmServo.attach(BACK_SERVO_PIN);
  LS_Servo.attach(LS_SERVO_PIN);
  frontSpringServo.attach(SPRING_SERVO1_PIN);
  backSpringServo.attach(SPRING_SERVO2_PIN);

  LS_Servo.write(0);
  frontArmServo.write(30);
  backArmServo.write(30);
  delay(1000);

#ifndef NO_SPRING_ARMS
  frontSpringServo.write(0);
  backSpringServo.write(0);

  *kill_switch_active = true;
  stopAllMotors();

  while (*kill_switch_active) handleKillSwitch(kill_switch_active);
  frontSpringServo.write(20);
  *kill_switch_active = true;
  stopAllMotors();
  
  while (*kill_switch_active) handleKillSwitch(kill_switch_active);
  backSpringServo.write(25);
  
  frontArmServo.write(ARM_SERVO_UP_ANGLE);
  backArmServo.write(ARM_SERVO_UP_ANGLE);

  delay(1000);
#endif
}

void setLineSensorServo(bool extended) {
  LS_Servo.write(extended * 20);  //25
}

void stopAllMotors() {
  frontMotors.setSpeed(1, 0);
  frontMotors.setSpeed(2, 0);
  backMotors.setSpeed(1, 0);
  backMotors.setSpeed(2, 0);
  frontArmServo.detach();
  backArmServo.detach();
  LS_Servo.detach();
  // Intentionally NOT detatching the spring servos
}

void setLRSpeeds(int16_t left, int16_t right, bool invert_front) {
  frontMotors.setSpeed(1, -right * (invert_front ? -1 : 1));
  frontMotors.setSpeed(2, -left * (invert_front ? -1 : 1));
  backMotors.setSpeed(1, right);
  backMotors.setSpeed(2, left);
}


int lavapit_cross(unsigned long start_time, int16_t *left, int16_t *right) {
  unsigned long t = millis();
  if (t - start_time < LAVA_PIT_SERVO_MOVE_DELAY) {
    *left = *right = 0;
    // float interp = min(1, (float)(t - start_time) / 600);

    frontArmServo.write((ARM_SERVO_DOWN_ANGLE_F - 5));
    backArmServo.write(ARM_SERVO_DOWN_ANGLE_R);
    return 0;
  } else if (t - start_time < LAVA_PIT_SERVO_MOVE_DELAY + LAVA_PIT_CROSS_DURATION) {
    *left = *right = LAVA_PIT_CROSS_SPEED;
    if (t - start_time > LAVA_PIT_SERVO_MOVE_DELAY + 200) {
      frontArmServo.write(ARM_SERVO_DOWN_ANGLE_F + 10);
      // backArmServo.write(ARM_SERVO_DOWN_ANGLE_R + 10);
    }
    return 0;
  } else if (t - start_time < 2 * LAVA_PIT_SERVO_MOVE_DELAY + LAVA_PIT_CROSS_DURATION) {
    *left = *right = 0;
    frontArmServo.write(ARM_SERVO_UP_ANGLE);
    backArmServo.write(ARM_SERVO_UP_ANGLE);
    return 0;
  } else return 1;
}

void treadmill() {
  const int baseSpeed = 800;

  frontMotors.setSpeed(1, -baseSpeed);  // front right
  frontMotors.setSpeed(2, -baseSpeed);
  backMotors.setSpeed(1, baseSpeed);
  backMotors.setSpeed(2, baseSpeed);
}

int climbStairs(unsigned long start_time, int16_t *left, int16_t *right, bool *invert_front) {
  static int8_t stair_count;
  *invert_front = false;

  unsigned long t = millis();
  if (t - start_time < STAIR_SERVO_DELAY_DURATION) {
    *left = *right = 0;
    frontArmServo.write(110);
    return 0;
  } else if (t - start_time < STAIR_SERVO_DELAY_DURATION + STAIR_STEP_DURATION) {
    *left = STAIR_SPEED;
    *right = STAIR_SPEED;
    *invert_front = true;  // Run the front wheels in reverse for the step over motion
    return 0;
  } else if (t - start_time < 2 * STAIR_SERVO_DELAY_DURATION + STAIR_STEP_DURATION) {
    *left = *right = 0;
    frontArmServo.write(0);
    if (stair_count >= TALL_STAIR) backArmServo.write(100);
    return 0;
  } else if (t - start_time < 2 * STAIR_SERVO_DELAY_DURATION + STAIR_STEP_DURATION + STAIR_DRIVE_DURATION) {
    *left = STAIR_SPEED;
    *right = STAIR_SPEED;
    return 0;
  } else {
    backArmServo.write(0);
    stair_count++;

    return 1;
  }
}

int zipwire_extend_arms(unsigned long start_time, int16_t *left, int16_t *right) {
  unsigned long t = millis();
  if (t - start_time < ZIPWIRE_SERVO_DELAY) {
    frontArmServo.write(50);
    backArmServo.write(0);
    return 0;
  } else if (t - start_time < 2 * ZIPWIRE_SERVO_DELAY) {
    frontSpringServo.write(0);
    backSpringServo.write(0);
    return 0;
  } else if (t - start_time < 2.5 * ZIPWIRE_SERVO_DELAY) {
    frontArmServo.write(60);
  } else if (t - start_time < 2.5 * ZIPWIRE_SERVO_DELAY + ZIPWIRE_DRIVE_CLIFF_DELAY) {
    *left = *right = ZIPWIRE_DRIVE_CLIFF_SPEED;
    float interp = ((t - start_time) - 2.5 * ZIPWIRE_SERVO_DELAY) / ZIPWIRE_DRIVE_CLIFF_DELAY;
    backArmServo.write(40 * interp);
  }
  return 1;
}