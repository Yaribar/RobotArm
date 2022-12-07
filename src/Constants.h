/*
  Constants
  Author: T-Kuhn.
  Sapporo, October, 2018. Released into the public domain.
  */

#ifndef Constants_h
#define Constants_h
#include "Arduino.h"

#define STEPPERBASE_DIR_PIN    15
#define STEPPERBASE_STEP_PIN   2

#define STEPPER1_DIR_PIN    4
#define STEPPER1_STEP_PIN   5
#define STEPPER2_DIR_PIN    18
#define STEPPER2_STEP_PIN   19
#define STEPPER3_DIR_PIN    21
#define STEPPER3_STEP_PIN   22
#define STEPPER4_DIR_PIN    32
#define STEPPER4_STEP_PIN   33

#define BUTTON_PIN          23
#define NAN_ALERT_LED       26 // This LEDs sole purpose is to alarm the user that there was NaN result in the IK calculations.
#define BUTTON_COOLDOWN_CYCLES 5000

#define EXECUTING_ISR_CODE  25
#define BUTTONDIR 34
#define POT 27

#define PULSES_PER_REVOLUTION 2048

// NOTE: SineStepper and MoveBatch ids must be lower then MAX_NUM_OF_STEPPERS
#define MAX_NUM_OF_STEPPERS 10
#define MAX_NUM_OF_BATCHED_MOVES 100
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

#endif
