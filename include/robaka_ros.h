#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <SimplePID.h>

#ifndef _ROBAKA_ROS_H
#define _ROBAKA_ROS_H

///////////////////////////////////////////////////////////////////////////////

class Chassis;
class SimplePID;

class RosNode {
public:
    RosNode(Chassis& chassis);

    void loop();
    
    void rSubscriberCallback(const std_msgs::Float32& cmdMsg);
    void lSubscriberCallback(const std_msgs::Float32& cmdMsg);

private:
    // Change to 1,1,150,150 for Uno
    ros::NodeHandle_<ArduinoHardware, 6, 6, 150, 150> nh;

    const int ticksPerMeter;

    sensor_msgs::Range rangeMsg;
    ros::Publisher rangePublisher;
    const char* frameId = "/sonar";

    std_msgs::Int16 lWheelMsg;
    std_msgs::Int16 rWheelMsg;
    ros::Publisher  lWheelPublisher;
    ros::Publisher  rWheelPublisher;
    unsigned long lWheelLast;
    unsigned long rWheelLast;

    std_msgs::Float32 lWheelVelocityMsg;
    std_msgs::Float32 rWheelVelocityMsg;
    ros::Publisher lWheelVelocityPublisher;
    ros::Publisher rWheelVelocityPublisher;

    int lWheelTargetRate = 0;
    int rWheelTargetRate = 0;

    int leftMotorCmd = 0;
    int rightMotorCmd = 0;

    // source: https://github.com/merose/ROSRobotControl/blob/master/ROSRobotControl.ino
    ros::Subscriber<std_msgs::Float32> lWheelTargetSub;
    ros::Subscriber<std_msgs::Float32> rWheelTargetSub;

    unsigned long lastUpdate;
    unsigned long lastMotorCmdTime;
    
    const float Ku = .15;
    const float Tu = .1142857143;

    const float Kp = 0.6*Ku;
    const float Ki = 2*Kp/Tu;
    const float Kd = Kp*Tu/8;

    SimplePID leftController = SimplePID(Kp, Ki, Kd);
    SimplePID rightController = SimplePID(Kp, Ki, Kd);

    Chassis& chassis;
};

#endif
