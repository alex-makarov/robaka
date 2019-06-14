#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

///////////////////////////////////////////////////////////////////////////////
// ROS support
//ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 1, 1, 150, 150> nh;

sensor_msgs::Range rangeMsg;
ros::Publisher rangePublisher("sonar", &rangeMsg);
char frameId[] = "/sonar";   // global frame id string
unsigned long rangeTimer = 0;
