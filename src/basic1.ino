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

bool Autonomy = true;

volatile int Pings[N_Sonars];
volatile int Ping = 0;

unsigned long moveStarted = 0;
unsigned long stuckSince = 0;
int headingOnStart = 0;
volatile int Heading = 0;
int prevHeading = -1;
int turnProgress = 0;
int Throttle = DEFAULT_MOTOR_SPEED;

///////////////////////////////////////////////////////////////////////////////
void pf(const char *fmt, ... ){
        char buf[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 256, fmt, args);
        va_end (args);
//        Serial.print(buf);
}

void p(const char* string) {
//  Serial.println(string);
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

  p("Setup complete");
}

void readIMU() {
 // IMU
sensors_event_t accelEvent;
sensors_event_t magEvent;
sensors_event_t gyroEvent;
sensors_vec_t orientation;

#ifdef IMU_DEBUG
pf ("IMU [roll, pitch, heading, x/y/z] [");
#endif

accel.getEvent(&accelEvent);
mag.getEvent(&magEvent);

if (imu.fusionGetOrientation(&accelEvent, &magEvent, &orientation)) {
  #ifdef IMU_DEBUG
  // Serial.print(orientation.roll);
  // Serial.print(", ");
  // Serial.print(orientation.pitch);
  // Serial.print(", ");
  // Serial.print(orientation.heading);
  // Serial.print(", ");
  #endif
  Heading = (int)orientation.heading;

  if (Heading < 0)
  {
    Heading = 360 + Heading;
  }

  if (prevHeading != -1 && Heading == 0 && abs(prevHeading - Heading) > 20) {
    Heading = prevHeading; // ignore
  }

  prevHeading = Heading;
}

gyro.getEvent(&gyroEvent);
#ifdef IMU_DEBUG
// Serial.print(gyroEvent.gyro.x);
// Serial.print("/");
// Serial.print(gyroEvent.gyro.y);
// Serial.print("/");
// Serial.print(gyroEvent.gyro.z);
// pf("]\n");
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

bool obstacleDetected() {
  return Ping < PROXIMITY_THRESHOLD && Ping > 0;
}

unsigned long moveDuration() { 
  return millis() - moveStarted;
}

int turnAngle() { // negative: left, positive: right
  int diff = headingOnStart - Heading;
  int delta = 0; 

  if (diff > 180) {
    delta = 360 - abs(diff);  
  } else if(diff < (-180)){
    delta = - (360 - abs(diff));
  } else {
    delta = - diff;
  }

  return delta;
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

  if (stuck && stuckSince == 0) {
    stuckSince = now;
  } else if (!stuck) {
    stuckSince = 0;
  }

  return stuck && now - stuckSince >= STUCK_DECISION_THRESHOLD;
}

void printState() {
  pf ("E: [%lu %lu %lu %lu]\n", EncoderCounts[0], EncoderCounts[1], EncoderCounts[2], EncoderCounts[3]);
}
