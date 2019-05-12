#include <stdarg.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

/* 
LIBRARY PATCHES REQUIRED TO MAKE IT WORK

1. I2C - modification of Adafruit_L3GD20_U.h to correct the IDs as described:

  !! NOTE that Gyro is using non-standard address 0x69 instead of 0x6B and D3 id instead of D4 or D7
    I2C device found at address 0x69  
    // #define L3GD20_ADDRESS           (0x6B)        // 1101011
    #define L3GD20_ADDRESS           (0x69)        // 1101001
    ...
    // #define L3GD20_ID                (0xD4)
    // #define L3GD20H_ID               (0xD7)
    #define L3GD20_ID                (0xD3)
    #define L3GD20H_ID               (0xD7)

2. MotorDriver library requires header change to redefine the pin used for one motor:
// Note than motor shield pin 3 is remapped to arduino pin 10,
// requiring header change as described here https://samwedge.uk/posts/using-the-arduino-motor-controller-shield-with-two-external-interrupts/

*/

/*
  "TT Motor" encoder and wheel parameters

  Motors have optical encoders, emitting light through a disk with 20 slots.
  Wheel size: 65x27mm , 204.2mm circle length.
  
  20 slots, 40 state changes per full revolution.
  40 state changes = 204.2mm traveled
  1 state change = (204.2 / 40) = 5.1mm

  100 state changes = 2.5 full revolutions = 51.5 cm
  200 state changes = 5 full revolutions = 103 cm
*/


///////////////////////////////////////////////////////////////////////////////
// Configuration
#define MOVE_MOTORS 1

#undef IMU_DEBUG
//#define IMU_DEBUG

const int MAX_SONAR_DISTANCE  = 300; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERIAL_SPEED        = 9600;
const int BLUETOOTH_SPEED     = 9600;

const int MOTOR_REAR_LEFT   = 1;
const int MOTOR_REAR_RIGHT  = 4;
const int MOTOR_FWD_LEFT    = 2;
const int MOTOR_FWD_RIGHT   = 3;

const int BUTTON_STOP = 0;
const int BUTTON_START = 1;

const int PROXIMITY_THRESHOLD = 20; // cm
const int BACKWARD_DURATION = 1000; // us
const int MAX_TURN_DURATION = 5000; // us
const int STUCK_UPDATE_THRESHOLD = 500; // us
const int STUCK_DECISION_THRESHOLD = 1000; // us
const int DEFAULT_MOTOR_SPEED = 255; // x/255
const int DIVERSION_HEADING = 90; // degrees; how much to turn when faced an obstacle

// Pins used on Arduino UNO
// Bluetooth HM-10
const int BLUETOOTH_TX_PIN = 13;
const int BLUETOOTH_RX_PIN = A2;
// Echo sonar HC-SR04
const int TRIGGER_PIN = A1; // Arduino pin tied to trigger pin on the ultrasonic sensor.
const int ECHO_PIN    = A0; // Arduino pin tied to echo pin on the ultrasonic sensor.
// IMU 9DOF
const int XMCS_PIN  = A5;
const int GCS_PIN   = A4;
// Encoders
const int ENCODER_1_PIN = 2; // REAR RIGHT. Interrupt pin.
const int ENCODER_4_PIN = 3; // REAR LEFT. Interrupt pin.
const int ENCODER_2_PIN = 9;  // FRONT RIGHT. UNUSED.
const int ENCODER_3_PIN = A3; // FRONT LEFT. UNUSED.
const int ENCODER_RIGHT_PIN = ENCODER_1_PIN;
const int ENCODER_LEFT_PIN = ENCODER_4_PIN;

// Rest pins are used for motors: 10 (instead of 3, see above),5,6,11; 4,7,8,12; 

///////////////////////////////////////////////////////////////////////////////
// Static objects
Adafruit_9DOF imu = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE); // NewPing setup of pins and maximum distance.
SoftwareSerial Bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
ArduinoBlue Phone(Bluetooth); // pass reference of Bluetooth object to ArduinoCommander.
MotorDriver m; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

///////////////////////////////////////////////////////////////////////////////
// Global variables
const int Encoders[] = {ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN};
volatile unsigned long EncoderUpdates[] = {0,0};
volatile unsigned long EncoderCounts[] = {0,0};
const int N_Encoders = 2;
const int LeftEncoderIndex = 0;
const int RightEncoderIndex = 1;

int sliderVal, button, sliderId;
int Ping = 0;
unsigned long moveStarted = 0;
unsigned long stuckSince = 0;
int headingOnStart = 0;
int Heading = 0;
int turnProgress = 0;
int Throttle = DEFAULT_MOTOR_SPEED;

enum State {
  Stop = 0,
  Forward,
  Backward,
  RightTurn,
  LeftTurn
} state;

const char *PrintableStates[] = {
  "STOP",
  "FORWARD",
  "BACKWARD",
  "RIGHT",
  "LEFT"
};

///////////////////////////////////////////////////////////////////////////////
void pf(const char *fmt, ... ){
        char buf[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 256, fmt, args);
        va_end (args);
        Serial.print(buf);
}

void p(const char* string) {
  Serial.println(string);
}

void pphone(const char *fmt, ... ) {
        char buf[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 256, fmt, args);
        va_end (args);

        Phone.sendMessage(String(buf));
}

///////////////////////////////////////////////////////////////////////////////
void setupIMU() {
  if (!gyro.begin()) {
    p("Oops ... unable to initialize the Gyroscope. Check your wiring!");
    while (1);
  }

  // Try to initialise and warn if we couldn't detect the chip
  if (!imu.begin() || !accel.begin() || !mag.begin()) { 
    p("Oops ... unable to initialize the IMU. Check your wiring!");
    while (1);
  }
    
  p("Found LSM303DLHC and L3GD20 IMU");
}

///////////////////////////////////////////////////////////////////////////////
void setupEncoders() {
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  pinMode(ENCODER_LEFT_PIN, INPUT);
  attachInterrupts();
}

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  p("Hello I am Robaka. Gav.");

  Bluetooth.begin(BLUETOOTH_SPEED);

  setupEncoders();
  setupIMU();
  state = Stop;

  p("Setup complete");
}

void leftEncoderEvent() {
  ++EncoderCounts[LeftEncoderIndex];
  EncoderUpdates[LeftEncoderIndex] = millis();
}

void rightEncoderEvent() {
  ++EncoderCounts[RightEncoderIndex];
  EncoderUpdates[RightEncoderIndex] = millis();
}

void readPhone()
{
  // ID of the button pressed pressed.
  button = Phone.getButton();

  // Returns the text data sent from the Phone.
  // After it returns the latest data, empty string "" is sent in subsequent.
  // calls until text data is sent again.
  String str = Phone.getText();
  if (str != "") {
    p(str.c_str());
  }

  // ID of the slider moved.
  sliderId = Phone.getSliderId();

  // Slider value goes from 0 to 200.
  sliderVal = Phone.getSliderVal();

  // Display button data whenever its pressed.
  if (button != -1) {
    pf("Button %d\n", button);
  }

  // Display slider data when slider moves
  if (sliderId != -1) {
    pf("Slider[%d]: %d\n", sliderId, sliderVal);
    Throttle = map(sliderVal, 0, 200, 0, 255);
  }
}

void readIMU() {
 // IMU
sensors_event_t event;
sensors_vec_t orientation;

#ifdef IMU_DEBUG
pf ("IMU [roll, pitch, heading, x/y/z] [");
#endif

accel.getEvent(&event);
if (imu.accelGetOrientation(&event, &orientation)) {
  #ifdef IMU_DEBUG
  Serial.print(orientation.roll);
  Serial.print(", ");
  Serial.print(orientation.pitch);
  Serial.print(", ");
  #endif
}

mag.getEvent(&event);
if (imu.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation)) {
  #ifdef IMU_DEBUG
  Serial.print(orientation.heading);
  Serial.print(", ");
  #endif
  Heading = orientation.heading;
}

gyro.getEvent(&event);
#ifdef IMU_DEBUG
Serial.print(event.gyro.x);
Serial.print("/");
Serial.print(event.gyro.y);
Serial.print("/");
Serial.print(event.gyro.z);
pf("]\n");
#endif
}

void readSonars() {
    Ping = sonar.ping_cm();
    if (Ping != 0) {
//      pf("[SONAR] %d\n", Ping);
    }
}

void detachInterrupts() {
  detachInterrupt(0);
  detachInterrupt(1);
}

void attachInterrupts() {
  attachInterrupt(0, rightEncoderEvent, CHANGE);
  attachInterrupt(1, leftEncoderEvent, CHANGE);
}

// Put your main code here, to run repeatedly:
void loop() {
  detachInterrupts();

  printState();

  State prevState = state;
  int prevThrottle = Throttle;

  readPhone();
  readSonars();
  readIMU();

  // //REMOVEME
  // if (state == Forward) {
  //   pf("L %lu R %lu\n", leftEncoderCount, rightEncoderCount);
  //   unsigned long avg = (leftEncoderCount + rightEncoderCount)/2;
  //   if (avg >= 400) {
  //     pf("---L %lu R %lu\n", leftEncoderCount, rightEncoderCount);
  //     state = Stop;
  //     leftEncoderCount = rightEncoderCount = 0;
  //   }
  // }


  // State machine
  step();

  if (state != prevState || Throttle != prevThrottle) {
    updateMotorsOnStep();
  }

  attachInterrupts();
}

bool obstacleDetected() {
  return Ping < PROXIMITY_THRESHOLD && Ping > 0;
}

State chooseStateForDiversion() {
  return random(0,2) > 0 ? RightTurn : LeftTurn;
}

unsigned long moveDuration() { 
  return millis() - moveStarted;
}

void step() {
  State prevState = state;

  switch (state) {
    case Forward:
      if (obstacleDetected() || isStuck()) {
        state = Backward;
        moveStarted = millis();
      }
     break;

    case Backward:
      if (moveDuration() >= BACKWARD_DURATION) {
        state = chooseStateForDiversion();
        headingOnStart = Heading;
        moveStarted = millis();
        turnProgress = 0;
      }
      break;

    case RightTurn:
    {
      turnProgress = ((360+Heading)-headingOnStart) % 360;
      // Ignore turn in the wrong direction -- happens at the early stage of the turn.
      if (turnProgress > 180) 
        break;
      if (turnProgress >= DIVERSION_HEADING || moveDuration() >= MAX_TURN_DURATION) {
        pf("RT, TP %d, Heading %d, headingOnStart %d, tdiff=%d\n", turnProgress, Heading, headingOnStart,moveDuration());
        state = Forward;
        moveStarted = millis();
      }
    }
    break;

    case LeftTurn:
    {
      turnProgress = ((360+headingOnStart)-Heading) % 360;
      // Ignore turn in the wrong direction -- happens at the early stage of the turn
      if (turnProgress > 180) 
        break;

      if (turnProgress >= DIVERSION_HEADING || moveDuration() >= MAX_TURN_DURATION) {
        pf("LT, TP %d, Heading %d, headingOnStart %d, tdiff=%d\n", turnProgress, Heading, headingOnStart,moveDuration());
        state = Forward;
        moveStarted = millis();
      }
    }
    break;

    case Stop:
      if (button == BUTTON_START) {
        state = Forward;
        resetEncoders();
        moveStarted = millis();
      }
      break;

    default: 
      break;
  }

  if (button == BUTTON_STOP) {
    state = Stop;
    moveStarted = millis();
  }

  if (prevState != state) {
   pf("State: %s -> %s\n", PrintableStates[prevState], PrintableStates[state]);
  }
}

void resetEncoders() {
  for (int i = 0; i < N_Encoders; i++) {
    EncoderCounts[i] = 0;
    EncoderUpdates[i] = millis();
  }
}

void updateMotorsOnStep() {
#if (MOVE_MOTORS == 0) 
  return;
#endif

  int speed = Throttle;

  switch (state) {
    case Forward:
      moveMotor(MOTOR_FWD_LEFT, FORWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, FORWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, FORWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, FORWARD, speed);
      break;

    case Backward:
      moveMotor(MOTOR_FWD_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, BACKWARD, speed);
      break;

    case RightTurn:
      moveMotor(MOTOR_FWD_LEFT, FORWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, FORWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, BACKWARD, speed);
      break;

    case LeftTurn:
      moveMotor(MOTOR_FWD_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, FORWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, FORWARD, speed);
      break;

    case Stop:
      moveMotor(MOTOR_FWD_LEFT, BRAKE, 0);
      moveMotor(MOTOR_REAR_LEFT, BRAKE, 0);
      moveMotor(MOTOR_FWD_RIGHT, BRAKE, 0);
      moveMotor(MOTOR_REAR_RIGHT, BRAKE, 0);
      break;

    default:
    break;
  }
}

void moveMotor(int motorId, int direction, int speed) {
  // Mapping according to motor orientation in the chassis
  switch(motorId) {
    case MOTOR_REAR_RIGHT:
    case MOTOR_FWD_RIGHT:
      if (direction == FORWARD || direction == BACKWARD) {
        // Only remap motor direction for rotation, not for braking or coasting
        m.motor(motorId, direction == FORWARD ? BACKWARD : FORWARD, speed);
      } else {
        m.motor(motorId, direction, speed);
      }
      break;
    default:
      m.motor(motorId, direction, speed);
      break;
  }
}

bool isStuck() {
  unsigned long now = millis();
  int wheelsStuck = 0;

  for (int i = 0; i < N_Encoders; i++) {
    if (now - EncoderUpdates[i] >= STUCK_UPDATE_THRESHOLD)
      ++wheelsStuck;
  }  

  bool stuck = wheelsStuck > 1;

  if (stuck && stuckSince == 0) {
    stuckSince = now;
  } else if (!stuck) {
    stuckSince = 0;
  }

  return stuck && now - stuckSince >= STUCK_DECISION_THRESHOLD;
}

void printState() {
//  pf ("E: [%lu %lu %lu %lu]\n", EncoderCounts[0], EncoderCounts[1], EncoderCounts[2], EncoderCounts[3]);
}
