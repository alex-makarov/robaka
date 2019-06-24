#include "robaka_ros.h"
#include "chassis.h"
#include "config.h"

//TODO - FIXME (upgrade rosserial_arduino to 0.8.0)

void lWheelTargetCallback(const std_msgs::Float32& cmdMsg) {


}

void rWheelTargetCallback(const std_msgs::Float32& cmdMsg) {


}


RosNode :: RosNode(Chassis& _chassis)
 :  rangePublisher("sonar", &rangeMsg),
    lWheelPublisher("lwheel", &lWheelMsg),
    rWheelPublisher("rwheel", &rWheelMsg),
    lWheelVelocityPublisher("lwheel_velocity", &lWheelVelocityMsg),
    rWheelVelocityPublisher("rwheel_velocity", &rWheelVelocityMsg),
    lWheelTargetSub("lwheel_target", &lWheelTargetCallback),
    rWheelTargetSub("rwheel_target", &rWheelTargetCallback),
    chassis(_chassis) {
  nh.initNode();
  nh.advertise(rangePublisher);
  nh.advertise(lWheelPublisher);
  nh.advertise(rWheelPublisher);
  nh.advertise(lWheelVelocityPublisher);
  nh.advertise(rWheelVelocityPublisher);

  rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeMsg.header.frame_id = frameId;
  rangeMsg.field_of_view = SONAR_FOV;
  rangeMsg.min_range = SONAR_MIN;
  rangeMsg.max_range = SONAR_MAX;

  lastUpdate = micros();
}

void RosNode::publishUpdates() {
    unsigned long now = micros();

    rangeMsg.range = chassis.range()/100.0;
    rangeMsg.header.stamp = nh.now();
    rangePublisher.publish(&rangeMsg);


    unsigned long lWheel = 0.5*((int)chassis.encoderCount(FrontLeft) + (int)chassis.encoderCount(RearLeft));
    unsigned long rWheel = 0.5*((int)chassis.encoderCount(FrontRight) + (int)chassis.encoderCount(RearRight));

    lWheelMsg.data = lWheel;
    rWheelMsg.data = rWheel;
    lWheelPublisher.publish(&lWheelMsg);
    rWheelPublisher.publish(&rWheelMsg);

    float dt = (now - lastUpdate) / 1E6;
    float lWheelRate = (lWheel - lWheelLast) / dt;
    float rWheelRate = (rWheel - rWheelLast) / dt;

    lWheelVelocityMsg.data = lWheelRate / TICKS_PER_METER;
    rWheelVelocityMsg.data = rWheelRate / TICKS_PER_METER;
    lWheelVelocityPublisher.publish(&lWheelVelocityMsg);
    rWheelVelocityPublisher.publish(&rWheelVelocityMsg);

    lWheelLast = lWheel;
    rWheelLast = rWheel;
    nh.spinOnce();
}
