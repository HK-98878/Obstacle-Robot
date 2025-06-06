#include "LineFollow.h"

#include "globals.h"

// ----------- Line Sensor Setup ------------
const uint8_t leftPins[9] = {53, 51, 49, 47, 45, 43, 41, 39, 37};
const uint8_t rightPins[9] = {44, 2, 46, 14, 19, 15, 18, 16, 17};
const uint8_t LED_PIN_LEFT = 35;
const uint8_t LED_PIN_RIGHT = 3;

#ifdef MANUAL_LINE_CALIBRATION
uint16_t mid[18];
#else
uint16_t mid[18] = {957, 797, 597, 652, 670, 558, 528, 524, 520,
                    546, 503, 498, 580, 595, 681, 786, 741, 846};
#endif

const uint16_t TIMEOUT_US = 3000;
const uint16_t CALIB_TIMES = 1200;

// -------------------------------------------

uint16_t readRC(uint8_t pin);
void readSensors(const uint8_t pins[], uint16_t raw[9]);
void calibrateSensors(const uint8_t pins[], uint8_t offset);
void readLine(float *h_offset, int8_t *l_width, uint8_t *flags);

// -------------------------------------------

void setupLineSensors() {
  pinMode(LED_PIN_LEFT, OUTPUT);
  pinMode(LED_PIN_RIGHT, OUTPUT);
  digitalWrite(LED_PIN_LEFT, HIGH);
  digitalWrite(LED_PIN_RIGHT, HIGH);

#ifdef MANUAL_LINE_CALIBRATION
  calibrateSensors(rightPins, 0);
  calibrateSensors(leftPins, 9);

#ifdef SERIAL_LOGGING
  Serial.println("Calibration Values:");
  for (uint8_t i = 0; i < 18; ++i) {
    Serial.print(mid[i]);
    Serial.print(", ");
  }
  Serial.println();
#endif
#endif
}

#define signed_constrain(amt, low, high)                                       \
  constrain(abs(amt), low, high) * (amt >= 0 ? 1 : -1)

int lineFollow(int16_t *left, int16_t *right) {
  static float err = 0, prevErr = 0;
  static uint8_t lineEventSequencePtr = 0, lineEventTriggered = 0;

  float h_offset;
  int8_t l_width;
  uint8_t flags;
  readLine(&h_offset, &l_width, &flags);

  CONDITIONAL_SERIAL_PRINT("Event: ");
  CONDITIONAL_SERIAL_PRINT(lineEventSequencePtr);
  CONDITIONAL_SERIAL_PRINT(" | Triggered: ");
  CONDITIONAL_SERIAL_PRINTLN(lineEventTriggered);

  if (!(flags & LINE_FLAGS_NO_LINE))
    err = h_offset; // Only update the error value if a line is detected

  static unsigned long start_time;
  // End of sequence
  if (lineEventSequencePtr >= LINE_EVENT_COUNT) {
    *left = *right = 0;
    return 1;
  }

  else if (!lineEventTriggered) { // Proceeding toward the next event, so use
                                  // standard controller
    if (flags & LINE_FLAGS_NO_LINE)
      return 0; // No line detected - exit without updating motor speeds
                // (accounts for gaps)

    int8_t dE = err - prevErr;
    int16_t controllerOutput = Kp_Line * err + Kd_Line * dE;

    // Set a base forward speed, unless a more significant turn is needed
    *left = *right = (abs(err) < LINE_TURN_THRESHOLD) * lineFollowBaseSpeed;

    *left = signed_constrain(*left - controllerOutput, lineFollowMinSpeed,
                             lineFollowMaxSpeed);
    *right = signed_constrain(*right + controllerOutput, lineFollowMinSpeed,
                              lineFollowMaxSpeed);

    // Check trigger conditions
    switch (lineEventSequence[lineEventSequencePtr]) {
    case EndOfLine:
      if (flags & LINE_FLAGS_FULL_WIDTH) {
        ++lineEventSequencePtr; // Event triggered, go to next and return 1 (to
                                // signify end)
        *left = *right = 0;
        return 1;
      }
      break;
    case ThickLine:
      if (abs(l_width) <= 5 && abs(h_offset < 1)) {
        ++lineEventSequencePtr;
      }
      break;

    case YJunction_Left:
    case YJunction_Right:
      if (flags & LINE_FLAGS_SPLIT) {
        start_time = millis();
        lineEventTriggered = 1; // Initiate turn
      }
      break;
    case YJunction_Straight:
      if (l_width >= 7) {
        start_time = millis();
        lineEventTriggered = 1; // Initiate turn
      }
      break;
    case SharpCorner_Left:
      if (!(flags & LINE_FLAGS_SPLIT) &&
          l_width >= 7) // Wide line left of centre, reaching the end of the
                        // sensor array
        lineEventTriggered = 1; // Initiate turn
      start_time = millis();
      break;
    case SharpCorner_Right:
      if (!(flags & LINE_FLAGS_SPLIT) &&
          l_width >= 7) // Wide line left of centre, reaching the end of the
                        // sensor array
        lineEventTriggered = 1; // Initiate turn
      start_time = millis();
      break;
    }
  }

  else { // Carrying out pre-programmed moves based on the event
    switch (lineEventSequence[lineEventSequencePtr]) {
    case YJunction_Left:
      *left = -lineFollowBaseSpeed; // Turn left
      *right = lineFollowBaseSpeed;

      if (lineEventTriggered == 1 && millis() - start_time > 1000)
        lineEventTriggered = 2;
      else if (lineEventTriggered == 2) {
        if (!(flags & LINE_FLAGS_SPLIT) &&
            abs(err) < LINE_TURN_THRESHOLD) { // No longer sees a split line,
                                              // and reasonably close to the
                                              // centre of the line
          lineEventTriggered = 0;
          ++lineEventSequencePtr;
        }
      }
      break;
    case YJunction_Right:
      *left = lineFollowBaseSpeed; // Turn right
      *right = -lineFollowBaseSpeed;

      if (lineEventTriggered == 1 && millis() - start_time > 1000)
        lineEventTriggered = 2;
      else if (lineEventTriggered == 2) {
        if (!(flags & LINE_FLAGS_SPLIT) &&
            abs(err) < LINE_TURN_THRESHOLD) { // No longer sees a split line,
                                              // and reasonably close to the
                                              // centre of the line
          lineEventTriggered = 0;
          ++lineEventSequencePtr;
        }
      }
      break;
    case YJunction_Straight:
      *left = *right = lineFollowBaseSpeed;
      if (abs(l_width) <= 8) {
        // lineEventTriggered = 2;
        // start_time = millis();
        lineEventTriggered = 0;
        ++lineEventSequencePtr;
      }
      if (lineEventTriggered == 2 && millis() - start_time > 500) {
        lineEventTriggered = 0;
        ++lineEventSequencePtr;
      }
      break;
    case SharpCorner_Left:
      if (lineEventTriggered == 1) { // Continue forward until the line ends
        *left = -lineFollowMinSpeed;
        *right = -lineFollowMinSpeed;

        if (l_width <= 6) {
          lineEventTriggered = 2;
          start_time = millis();
        }
      } else if (lineEventTriggered == 2) { // Fixed duration initial turn
        if (millis() - start_time < 500) {
          *left = -lineFollowTurnSpeed / 2;
          *right = lineFollowTurnSpeed;
        } else
          lineEventTriggered = 3;
      } else if (lineEventTriggered ==
                 3) { // Continue turn until we re-establish the line
        *left = -lineFollowTurnSpeed / 2;
        *right = lineFollowTurnSpeed;

        if (abs(l_width) <= 6) { // && abs(err) < 7 - (float)l_width / 2
          lineEventTriggered = 0;
          ++lineEventSequencePtr;
        }
      }
      break;
    case SharpCorner_Right: // As above, but inverted directions

      if (lineEventTriggered == 1) {
        *left = -lineFollowMinSpeed;
        *right = -lineFollowMinSpeed;

        if (millis() - start_time > 150) {
          lineEventTriggered = 2;
          start_time = millis();
        }
      } else if (lineEventTriggered == 2) {
        if (millis() - start_time < 500) {
          *left = lineFollowTurnSpeed;
          *right = -lineFollowTurnSpeed / 2;
        } else
          lineEventTriggered = 3;
      } else if (lineEventTriggered == 3) {
        *left = lineFollowTurnSpeed;
        *right = -lineFollowTurnSpeed / 2;

        if (abs(l_width) <= 6) { // && abs(err) < 7 - (float)l_width / 2
          lineEventTriggered = 0;
          ++lineEventSequencePtr;
        }
      }
      break;
    }
  }

  prevErr = err;
  return 0;
}

#pragma region Private Functions

uint16_t readRC(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  uint32_t start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > TIMEOUT_US)
      return TIMEOUT_US;
  }
  return micros() - start;
}

void readSensors(const uint8_t pins[], uint16_t raw[9]) {
  for (uint8_t i = 0; i < 9; i++) {
    raw[i] = readRC(pins[i]);
  }
}

void calibrateSensors(const uint8_t pins[], uint8_t offset) {
  uint16_t mn[9], mx[9];
  for (uint8_t i = 0; i < 9; i++) {
    mn[i] = TIMEOUT_US;
    mx[i] = 0;
  }
  uint16_t raw[9];
  for (uint16_t n = 0; n < CALIB_TIMES; n++) {
    readSensors(pins, raw);
    for (uint8_t i = 0; i < 9; i++) {
      mn[i] = min(mn[i], raw[i]);
      mx[i] = max(mx[i], raw[i]);
    }
    delay(2);
  }
  for (uint8_t i = 0; i < 9; i++)
    mid[i + offset] = (mn[i] + mx[i]) / 2;
}

void readLine(float *h_offset, int8_t *l_width, uint8_t *flags) {
  int8_t min = 127;
  int8_t max = -1;
  *flags = LINE_FLAGS_NO_LINE | LINE_FLAGS_FULL_WIDTH;

  uint8_t white_detected = 0;

  int8_t line_ended = 0;

  uint16_t raw[9];

  // Right
  readSensors(rightPins, raw);
  for (uint8_t i = 0; i < 9; i++) {
    // Dark colour detected
    // Serial.print(raw[i] >= mid[i]);
    if (raw[i] >= mid[i]) {
      *flags &= ~LINE_FLAGS_NO_LINE;
      if (i < min)
        min = i;
      if (i > max) {
        max = i;
        if (line_ended) // Line re-detected after thinking it ended
          *flags |= LINE_FLAGS_SPLIT;
      }
    }
    // Light colour detected  after a line has been detected
    //  - i.e. other edge of the line
    // Flag is used for detecting double lines
    else {
      ++white_detected;
      if (max != -1 && i > max)
        line_ended = 1;
    }
  }

  // Left - same logic as before, but adjusted indexes
  readSensors(leftPins, raw);
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t i_adjusted = i + 9;
    // Serial.print(raw[i] >= mid[i_adjusted]);
    if (raw[i] >= mid[i_adjusted]) {
      *flags &= ~LINE_FLAGS_NO_LINE;
      if (i_adjusted < min)
        min = i_adjusted;
      if (i_adjusted > max) {
        max = i_adjusted;
        if (line_ended)
          *flags |= LINE_FLAGS_SPLIT;
      }
    } else {
      ++white_detected;
      if (max != -1 && i_adjusted > max)
        line_ended = 1;
    }
  }
  // Serial.println();

  *h_offset = (float)(min + max) / 2 - 8;
  *l_width = max - min;

  if (white_detected > 1)
    *flags &= ~LINE_FLAGS_FULL_WIDTH;

#ifdef SERIAL_LOGGING
  Serial.print("H-offset: ");
  Serial.print(*h_offset);
  Serial.print(" | L-width: ");
  Serial.print(*l_width);
  Serial.print(" | Flags:");
  Serial.println(*flags);
#endif
}

#pragma endregion
