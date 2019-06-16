#include <stdarg.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

#include "config.h"
#include "robaka_ros.h"
#include "chassis.h"

#define vLog Serial.println
#ifndef vLog
  #define vLog(x)
#endif

Chassis* robaka;

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  setupROS();

  robaka = Chassis::instance();
  
  if (robaka->init()) {
    vLog("Init complete");
  } else {
    vLog("Init failed");
  }
}

// MAIN LOOP
void loop() {
  robaka->updateSensors();
  updateROS();
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

void updateROS() {
  if ((millis() - rangeTimer) > RANGE_UPDATE_INTERVAL) {
//    rangeMsg.range = Ping/100.0;
    rangeMsg.header.stamp = nh.now();
    rangePublisher.publish(&rangeMsg);
    rangeTimer = millis() + RANGE_UPDATE_INTERVAL;
  }

  nh.spinOnce();
}
