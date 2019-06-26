#include "robaka_ros.h"
#include "chassis.h"
#include "config.h"
#include "utils.h"

// FIXME: remove this as soon as rosserial_arduino>=0.8.0 is avaialable,
// which supports member functions for callbacks

static RosNode* _rosnode = NULL;

void lWheelTargetCallback(const std_msgs::Float32& cmdMsg) {
	if (_rosnode) {
		_rosnode->lSubscriberCallback(cmdMsg);
	}
}

void rWheelTargetCallback(const std_msgs::Float32& cmdMsg) {
	if (_rosnode) {
		_rosnode->rSubscriberCallback(cmdMsg);
	}
}

RosNode :: RosNode(Chassis& _chassis)
	:  ticksPerMeter(TICKS_PER_METER),
	   rangePublisher("sonar", &rangeMsg),
	   lWheelPublisher("lwheel", &lWheelMsg),
	   rWheelPublisher("rwheel", &rWheelMsg),
	   imuPublisher("imu_data", &imuMsg),
	   lWheelVelocityPublisher("lwheel_velocity", &lWheelVelocityMsg),
	   rWheelVelocityPublisher("rwheel_velocity", &rWheelVelocityMsg),
	   lWheelTargetSub("lwheel_vtarget", &lWheelTargetCallback),
	   rWheelTargetSub("rwheel_vtarget", &rWheelTargetCallback),
	   chassis(_chassis) {


//	vLog(F("1"));	
	return;
	nh.initNode();
	vLog("2");	
	broadcaster.init(nh);
	vLog("3");

	nh.advertise(rangePublisher);
	nh.advertise(lWheelPublisher);
	nh.advertise(rWheelPublisher);
	nh.advertise(lWheelVelocityPublisher);
	nh.advertise(rWheelVelocityPublisher);
	nh.advertise(imuPublisher);

	nh.subscribe(lWheelTargetSub);
	nh.subscribe(rWheelTargetSub);

	rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	rangeMsg.header.frame_id = frameId;
	rangeMsg.field_of_view = SONAR_FOV;
	rangeMsg.min_range = SONAR_MIN;
	rangeMsg.max_range = SONAR_MAX;

	//FIXME
	if (_rosnode != NULL) {
		vLog(F("Multiple ROS nodes are not allowed"));
		abort();
	}
	_rosnode = this;

	while(!nh.connected()) {
		nh.spinOnce();
	}

	lastUpdate = micros();
	lastMotorCmdTime = millis();
}

void RosNode::loop() {
    unsigned long now = micros();

    rangeMsg.range = chassis.range()/100.0;
    rangeMsg.header.stamp = nh.now();
    rangePublisher.publish(&rangeMsg);

	t.header.frame_id = imuFrameId;
	t.child_frame_id = childFrameId;
	t.transform.translation.x = 0.5;
	t.transform.rotation.x = chassis.orientation().x;
	t.transform.rotation.y = chassis.orientation().y;
	t.transform.rotation.z = chassis.orientation().z;
	t.transform.rotation.w = 0;
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	imuMsg.header.stamp = nh.now();
	imuMsg.header.frame_id = imuFrameId;
	imuMsg.orientation.x = chassis.orientation().x;
	imuMsg.orientation.y = chassis.orientation().y;
	imuMsg.orientation.z = chassis.orientation().z;
	imuMsg.angular_velocity.x = chassis.gyro().x;
	imuMsg.angular_velocity.y = chassis.gyro().y;
	imuMsg.angular_velocity.z = chassis.gyro().z;
	imuMsg.linear_acceleration.x = chassis.linearAcceleration().x;
	imuMsg.linear_acceleration.y = chassis.linearAcceleration().y;
	imuMsg.linear_acceleration.z = chassis.linearAcceleration().z;
	imuPublisher.publish(&imuMsg);

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


	int leftControl = leftController.getControlValue(lWheelRate, dt);
	leftMotorCmd += min(MAX_MOTOR_CMD, leftControl);
	leftMotorCmd = constrain(leftMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
	if (leftMotorCmd > 0) {
		leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
	}
  
	int rightControl = rightController.getControlValue(rWheelRate, dt);
	rightMotorCmd += min(MAX_MOTOR_CMD, rightControl);
	rightMotorCmd = constrain(rightMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
	if (rightMotorCmd > 0) {
		rightMotorCmd = max(rightMotorCmd, MIN_MOTOR_CMD);
	}

	// Coast to a stop if target is zero.
	if (lWheelTargetRate == 0) {
		leftMotorCmd = 0;
	}
	if (rWheelTargetRate == 0) {
		rightMotorCmd = 0;
	}

	chassis.moveMotor(FrontLeft, leftMotorCmd);
	chassis.moveMotor(RearLeft, leftMotorCmd);
	chassis.moveMotor(FrontRight, rightMotorCmd);
	chassis.moveMotor(RearRight, rightMotorCmd);

    lWheelLast = lWheel;
    rWheelLast = rWheel;
    nh.spinOnce();
}

void RosNode::lSubscriberCallback(const std_msgs::Float32& cmdMsg) {
	lastMotorCmdTime = millis();
	lWheelTargetRate = cmdMsg.data * ticksPerMeter;
	leftController.setSetPoint(lWheelTargetRate);
}

void RosNode::rSubscriberCallback(const std_msgs::Float32& cmdMsg) {
	lastMotorCmdTime = millis();
	rWheelTargetRate = cmdMsg.data * ticksPerMeter;
	rightController.setSetPoint(rWheelTargetRate);
}
