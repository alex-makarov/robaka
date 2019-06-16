#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

#include "chassis.h"
#include "config.h"

#ifdef ARDUINO_AVR_UNO
#include "hw_uno.h"
#elif ARDUINO_SAM_DUE
#include "hw_due.h"
#endif

#define vLog Serial.println
#ifndef vLog
  #define vLog(x)
#endif

class Chassis::HWImpl {
public:
    Adafruit_9DOF imu = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

    NewPing *sonars[N_Sonars];
    MotorDriver motor; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

    volatile int pings[N_Sonars];
    int heading;

    bool initIMU();
    bool initSonar();
    bool initEncoders();

    void readIMU();
    void readSonar();

    void attachInterrupts();
    void detachInterrupts();
};

bool Chassis::HWImpl::initIMU() {
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
    return true;
}

bool Chassis::HWImpl::initSonar() {
  for (int i = 0; i < N_Sonars; i+=2) {
    sonars[i] = new NewPing(SonarPins[i], SonarPins[i+1], MAX_SONAR_DISTANCE);
  }
  return true;
}

bool Chassis::HWImpl::initEncoders() {
  for (int i = 0; i < N_Encoders; i++) {
    pinMode(Encoders[i], INPUT);
  }
  attachInterrupts();
  return true;
}

void Chassis::HWImpl::attachInterrupts() {
 for (int i = 0; i < N_Encoders; i++) {
    attachInterrupt(digitalPinToInterrupt(Encoders[i]), EncoderISRs[i], CHANGE);
    pinMode(Encoders[i], INPUT);
  }
}

void Chassis::HWImpl::detachInterrupts() {
  for (int i = 0; i < N_Encoders; i++) {
      detachInterrupt(digitalPinToInterrupt(Encoders[i]));
  }
}

void Chassis::HWImpl::readIMU() {
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

    heading = (int)orientation.heading;
}

gyro.getEvent(&gyroEvent);
#ifdef IMU_DEBUG
vLog("Gyro x/y/z");
vLog(String(gyroEvent.gyro.x) + ", " +
    String(gyroEvent.gyro.y) + ", " +
    String(gyroEvent.gyro.z));
#endif
}

void Chassis::HWImpl::readSonar() {
  for (int i = 0; i < N_Sonars; i++) {
    pings[i] = sonars[i]->ping_cm();
  }
}

///////////////////////////////////////////////////////////////////////////////


Chassis* Chassis::_instance = 0;

Chassis* Chassis::instance() {
    if (_instance == 0) {
        _instance = new Chassis();
    }

    return _instance;
}

Chassis::Chassis() {
    impl = new Chassis::HWImpl();
}

Chassis::~Chassis() {
    delete impl;
}

bool Chassis::init() {
    return impl->initEncoders() &&
        impl->initSonar() &&
        impl->initIMU();
}

void Chassis::updateSensors() {
    impl->detachInterrupts();
    impl->readIMU();
    impl->readSonar();
    impl->attachInterrupts();
}

void Chassis::moveMotor(int motorId, int direction, int speed) {
  // Mapping according to motor orientation in the chassis
  switch(motorId) {
    case MOTOR_REAR_RIGHT:
    case MOTOR_FWD_RIGHT:
      if (direction == FORWARD || direction == BACKWARD) {
        // Only remap motor direction for rotation, not for braking or coasting
        impl->motor.motor(motorId, direction == FORWARD ? BACKWARD : FORWARD, speed);
      } else {
        impl->motor.motor(motorId, direction, speed);
      }
      break;
    default:
      impl->motor.motor(motorId, direction, speed);
      break;
  }
}


//TODO:
// bool isStuck() {
//   unsigned long now = millis();
//   int wheelsStuck = 0;

//   for (int i = 0; i < N_Encoders; i++) {
//     if (now - EncoderUpdates[i] >= STUCK_UPDATE_THRESHOLD)
//       ++wheelsStuck;
//   }  

//   bool stuck = wheelsStuck > 0;

//   if (stuck && StuckSince == 0) {
//     StuckSince = now;
//   } else if (!stuck) {
//     StuckSince = 0;
//   }

//   return stuck && now - StuckSince >= STUCK_DECISION_THRESHOLD;
// }
