///////////////////////////////////////////////////////////////////////////////
// Configuration
#define MOVE_MOTORS 1

#undef IMU_DEBUG
//#define IMU_DEBUG

const int MAX_SONAR_DISTANCE  = 300; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERIAL_SPEED        = 57600;
const int BLUETOOTH_SPEED     = 9600;

const int RANGE_UPDATE_INTERVAL = 50; // ms

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