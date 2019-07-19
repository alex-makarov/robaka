#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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
    void cmdvelCallback(const geometry_msgs::Twist& cmdMsg);

private:

    void publishSonar();
    void publishIMU();
    void handleOdometry();

    ros::NodeHandle nh;

    nav_msgs::Odometry odometryMsg;
    ros::Publisher odometryPublisher;

    sensor_msgs::Range rangeMsg;
    ros::Publisher leftRangePublisher;
    ros::Publisher middleRangePublisher;
    ros::Publisher rightRangePublisher;

    std_msgs::Int16 lWheelMsg;
    std_msgs::Int16 rWheelMsg;
    ros::Publisher  lWheelPublisher;
    ros::Publisher  rWheelPublisher;

    sensor_msgs::Imu imuMsg;
    ros::Publisher imuPublisher;

    sensor_msgs::MagneticField magMsg;
    ros::Publisher magPublisher;

    geometry_msgs::TransformStamped t;
    tf::TransformBroadcaster broadcaster;

    std_msgs::Float32 lWheelVelocityMsg;
    std_msgs::Float32 rWheelVelocityMsg;
    ros::Publisher lWheelVelocityPublisher; 
    ros::Publisher rWheelVelocityPublisher;

    const char* leftSonarFrameId = "sonarleft";
    const char* middleSonarFrameId = "sonarmiddle";
    const char* rightSonarFrameId = "sonarright";
    const char* baseFrameId = "base_link";
    const char* imuFrameId = "imu";    

    long lWheelLast = 0;
    long rWheelLast = 0; 
    int lWheelTargetRate = 0;
    int rWheelTargetRate = 0;
    int leftMotorCmd = 0;
    int rightMotorCmd = 0;

    // source: https://github.com/merose/ROSRobotControl/blob/master/ROSRobotControl.ino
    ros::Subscriber<std_msgs::Float32> lWheelTargetSub;
    ros::Subscriber<std_msgs::Float32> rWheelTargetSub;
    ros::Subscriber<geometry_msgs::Twist> cmdvelSub;

    unsigned long lastUpdate;
    unsigned long lastMotorCmdTime;
    ros::Time lastLoopTs;
    ros::Time rosTime;

    float x = 0, y = 0, z = 0;
    bool blinkState;

    const int ticksPerMeter;

    // PID controller parameters
    const float Kp = 0.07;
    const float Ki = 0.0;
    const float Kd = 0.001;
    SimplePID leftController = SimplePID(Kp, Ki, Kd);
    SimplePID rightController = SimplePID(Kp, Ki, Kd);

    int maxMotorSpeedParam;
    Chassis& chassis;
};

#endif
