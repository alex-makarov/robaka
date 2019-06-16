#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>

#ifndef _ROBAKA_ROS_H
#define _ROBAKA_ROS_H

///////////////////////////////////////////////////////////////////////////////

class Chassis;

class RosNode {
public:
    RosNode(Chassis& chassis);

    void publishUpdates();

private:
    ros::NodeHandle_<ArduinoHardware, 1, 1, 150, 150> nh;

    sensor_msgs::Range rangeMsg;
    ros::Publisher rangePublisher;
    const char* frameId = "/sonar";

    Chassis& chassis;
};

#endif
