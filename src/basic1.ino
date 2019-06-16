#include "config.h"
#include "robaka_ros.h"
#include "chassis.h"
#include "utils.h"

Chassis* robaka = Chassis::instance();
RosNode node(*robaka);

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  
  if (robaka->init()) {
    vLog(F("Init complete"));
  } else {
    vLog(F("Init failed"));
  }
}

// MAIN LOOP
void loop() {
  if (robaka->isInitialized()) {
    robaka->updateSensors();
    node.publishUpdates();
  }
}
