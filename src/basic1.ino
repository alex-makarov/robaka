#include "config.h"
#include "robaka_ros.h"
#include "chassis.h"
#include "utils.h"

Chassis* robaka = Chassis::instance();
RosNode* node; //(*robaka);

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
	Serial.begin(57600);
	while(!Serial);
	vLog(F("[OK] Power up"));

	if (robaka->init()) {
		vLog(F("[OK] HW init complete"));
	} else {
		vLog(F("[ERROR] HW init failed"));
	}

	vLog(F("[OK] Started, waiting for serial USB..."));
#ifdef ARDUINO_SAM_DUE
	SerialUSB.begin(57600);
	while (!SerialUSB);
#endif
	node = new RosNode(*robaka);
	vLog(F("[OK] ROS init complete"));
}

// MAIN LOOP
void loop() {
	if (robaka->isInitialized()) {
		robaka->updateSensors();
		node->loop();
	}
}
