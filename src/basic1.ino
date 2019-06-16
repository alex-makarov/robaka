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

Chassis* robaka = Chassis::instance();
RosNode node(*robaka);

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  
  if (robaka->init()) {
    vLog("Init complete");
  } else {
    vLog("Init failed");
  }
}

// MAIN LOOP
void loop() {
  robaka->updateSensors();
  node.publishUpdates();
}
