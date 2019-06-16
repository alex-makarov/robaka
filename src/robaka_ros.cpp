#include "robaka_ros.h"
#include "chassis.h"
#include "config.h"

RosNode :: RosNode(Chassis& _chassis)
 : rangePublisher("sonar", &rangeMsg), chassis(_chassis) {
  nh.initNode();
  nh.advertise(rangePublisher);

  rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeMsg.header.frame_id = frameId;
  rangeMsg.field_of_view = SONAR_FOV;
  rangeMsg.min_range = SONAR_MIN;
  rangeMsg.max_range = SONAR_MAX;
}

void RosNode::publishUpdates() {
    rangeMsg.range = chassis.range()/100.0;
    rangeMsg.header.stamp = nh.now();
    rangePublisher.publish(&rangeMsg);

    nh.spinOnce();
}
