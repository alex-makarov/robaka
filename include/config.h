#ifndef _CONFIG_H
#define _CONFIG_H

///////////////////////////////////////////////////////////////////////////////
// Configuration
#define MOVE_MOTORS 1

#undef IMU_DEBUG
//#define IMU_DEBUG

#undef SONAR_DEBUG
//#define SONAR_DEBUG

#undef ENCODER_DEBUG
//#define ENCODER_DEBUG

const int MAX_SONAR_DISTANCE  = 100; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERIAL_SPEED        = 9600; //57600;
const int BLUETOOTH_SPEED     = 9600;

const int RANGE_UPDATE_INTERVAL = 50; // ms

// Motor shield IDs
const int MOTOR_REAR_LEFT   = 1;
const int MOTOR_REAR_RIGHT  = 4;
const int MOTOR_FWD_LEFT    = 2;
const int MOTOR_FWD_RIGHT   = 3;

const int PROXIMITY_THRESHOLD = 20; // cm
const int BACKWARD_DURATION = 1000; // us
const int MAX_TURN_DURATION = 5000; // us
const int STUCK_UPDATE_THRESHOLD = 500; // us
const int STUCK_DECISION_THRESHOLD = 1000; // us
const int DEFAULT_MOTOR_SPEED = 255; // x/255
const int DIVERSION_HEADING = 90; // degrees; how much to turn when faced an obstacle

const int TICKS_PER_METER = 196;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 100;
// Maximum motor control value.
const int MAX_MOTOR_CMD = 255;

const float SONAR_FOV = 0.26;
const float SONAR_MIN = 0.03;
const float SONAR_MAX = 1.0;

#endif
