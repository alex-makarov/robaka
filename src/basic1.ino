#include <stdarg.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

#include "config.h"
#include "robaka_ros.h"

#ifdef ARDUINO_AVR_UNO
#include "hw_uno.h"
#elif ARDUINO_SAM_DUE
#include "hw_due.h"
#endif

#define vLog Serial.println
#ifndef vLog
  #define vLog(x)
#endif

///////////////////////////////////////////////////////////////////////////////
// Static objects
// TODO: FIX for Due
Adafruit_9DOF imu = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

NewPing *Sonars[N_Sonars];
MotorDriver m; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

///////////////////////////////////////////////////////////////////////////////
// Global variables

volatile int Pings[N_Sonars];
volatile int Ping = 0;
volatile int Heading = 0;

unsigned long StuckSince = 0;

///////////////////////////////////////////////////////////////////////////////
void setupIMU() {
  if (!gyro.begin()) {
    vLog("Oops ... unable to initialize the Gyroscope. Check your wiring!");
    while (1);
  }

  // Try to initialise and warn if we couldn't detect the chip
  if (!imu.begin() || !accel.begin() || !mag.begin()) { 
    vLog("Oops ... unable to initialize the IMU. Check your wiring!");
    while (1);
  }
    
  vLog("Found LSM303DLHC and L3GD20 IMU");
}

///////////////////////////////////////////////////////////////////////////////
void setupSonars() {
  for (int i = 0; i < N_Sonars; i+=2) {
    Sonars[i] = new NewPing(SonarPins[i], SonarPins[i+1], MAX_SONAR_DISTANCE);
  }
}

///////////////////////////////////////////////////////////////////////////////
void setupEncoders() {
  for (int i = 0; i < N_Encoders; i++) {
    pinMode(Encoders[i], INPUT);
  }
  attachInterrupts();
}

///////////////////////////////////////////////////////////////////////////////
void setupROS() {
  nh.initNode();
  nh.advertise(rangePublisher);

  rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeMsg.header.frame_id = frameId;
  rangeMsg.field_of_view = 0.26;
  rangeMsg.min_range = 0.03;
  rangeMsg.max_range = 1.0;
}

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  setupSonars();
  setupROS();
  setupEncoders();
  setupIMU();

  vLog("Setup complete");
}

void readIMU() {
 // IMU
sensors_event_t accelEvent;
sensors_event_t magEvent;
sensors_event_t gyroEvent;
sensors_vec_t orientation;

#ifdef IMU_DEBUG
vLog ("IMU [roll, pitch, heading, x/y/z]");
#endif

accel.getEvent(&accelEvent);
mag.getEvent(&magEvent);

if (imu.fusionGetOrientation(&accelEvent, &magEvent, &orientation)) {
  #ifdef IMU_DEBUG
  vLog(String(orientation.roll) + ", " +
  String(orientation.pitch) + ", " +
  String(orientation.heading));
  #endif

  Heading = (int)orientation.heading;
}

gyro.getEvent(&gyroEvent);
#ifdef IMU_DEBUG
vLog("Gyro x/y/z");
vLog(String(gyroEvent.gyro.x) + ", " +
    String(gyroEvent.gyro.y) + ", " +
    String(gyroEvent.gyro.z));
#endif
}

void readSonars() {
  for (int i = 0; i < N_Sonars; i++) {
    Pings[i] = Sonars[i]->ping_cm();
  }
  Ping = Pings[0]; // backward compat
}

void detachInterrupts() {
  for (int i = 0; i < N_Encoders; i++) {
      detachInterrupt(digitalPinToInterrupt(Encoders[i]));
  }
}

void attachInterrupts() {
  for (int i = 0; i < N_Encoders; i++) {
    attachInterrupt(digitalPinToInterrupt(Encoders[i]), EncoderISRs[i], CHANGE);
    pinMode(Encoders[i], INPUT);
  }
}

// MAIN LOOP
void loop() {
  detachInterrupts();

  readSonars();
  readIMU();

  updateROS();

  attachInterrupts();
}

void updateROS() {
  if ((millis() - rangeTimer) > RANGE_UPDATE_INTERVAL) {
    rangeMsg.range = Ping/100.0;
    rangeMsg.header.stamp = nh.now();
    rangePublisher.publish(&rangeMsg);
    rangeTimer = millis() + RANGE_UPDATE_INTERVAL;
  }

  nh.spinOnce();
}

void resetEncoders() {
  for (int i = 0; i < N_Encoders; i++) {
    EncoderCounts[i] = 0;
    EncoderUpdates[i] = millis();
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

  bool stuck = wheelsStuck > 0;

  if (stuck && StuckSince == 0) {
    StuckSince = now;
  } else if (!stuck) {
    StuckSince = 0;
  }

  return stuck && now - StuckSince >= STUCK_DECISION_THRESHOLD;
}