#include "robaka_ros.h"
#include "chassis.h"
#include "config.h"
#include "utils.h"
#include <tf/tf.h>

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
	:  	ticksPerMeter(TICKS_PER_METER),
		odometryPublisher("odom", &odometryMsg),
	   	leftRangePublisher("sonar", &rangeMsg),
		middleRangePublisher("sonar", &rangeMsg),
		rightRangePublisher("sonar", &rangeMsg),	   	   
	   lWheelPublisher("lwheel", &lWheelMsg),
	   rWheelPublisher("rwheel", &rWheelMsg),
	   imuPublisher("imu_data", &imuMsg),
	   lWheelVelocityPublisher("lwheel_velocity", &lWheelVelocityMsg),
	   rWheelVelocityPublisher("rwheel_velocity", &rWheelVelocityMsg),
	   lWheelTargetSub("lwheel_vtarget", &lWheelTargetCallback),
	   rWheelTargetSub("rwheel_vtarget", &rWheelTargetCallback),
	   chassis(_chassis) {

	nh.initNode();
	broadcaster.init(nh);

	nh.advertise(odometryPublisher);
	nh.advertise(leftRangePublisher);
	nh.advertise(middleRangePublisher);
	nh.advertise(rightRangePublisher);
	nh.advertise(lWheelPublisher);
	nh.advertise(rWheelPublisher);
	nh.advertise(lWheelVelocityPublisher);
	nh.advertise(rWheelVelocityPublisher);
	nh.advertise(imuPublisher);

	nh.subscribe(lWheelTargetSub);
	nh.subscribe(rWheelTargetSub);

	rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	rangeMsg.header.frame_id = leftSonarFrameId;
	rangeMsg.field_of_view = SONAR_FOV;
	rangeMsg.min_range = SONAR_MIN;
	rangeMsg.max_range = SONAR_MAX;

	//FIXME
	if (_rosnode != NULL) {
		vLog(F("Multiple ROS nodes are not allowed"));
		abort();
	}
	_rosnode = this;

	delay(1000);

//	while(!nh.connected()) {
//		nh.spinOnce();
//	}

	lastUpdate = micros();
	lastMotorCmdTime = millis();
}

void RosNode::loop() {

    unsigned long now = micros();

	// ODOMETRY /////

	float dt = (now - lastUpdate) / 1E6;
	float vx = chassis.speedMs();
	float vy = 0;
	float th = chassis.yaw();
	float deltaX = (vx*cos(th) - vy*sin(th)) * dt;
	float deltaY = (vx*sin(th) + vy*cos(th)) * dt;
	float deltaTh = chassis.gyro().z;

	x += deltaX;
	y += deltaY;

	vLog("dt " + String(dt) + "\n" +
		"vx " + String(vx) + " " +
		"th " + String(th) + " " +
		"dx " + String(deltaX) + " " + 
		"dy " + String(deltaY) + " " +
		"dTh " + String(deltaTh) + " " +
		"x " + String(x) + " " +
		"y " + String(y) + " "
	);


	geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";
	t.transform.translation.x = x;
	t.transform.translation.y = y;
	t.transform.translation.z = 0;
	t.transform.rotation = odom_quat;	
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	odometryMsg.header.stamp = nh.now();
	odometryMsg.header.frame_id = "odom";
	odometryMsg.pose.pose.position.x = x;
	odometryMsg.pose.pose.position.x = y;
	odometryMsg.pose.pose.position.x = z;
	odometryMsg.pose.pose.orientation = odom_quat;

	odometryMsg.child_frame_id = "base_link";
	odometryMsg.twist.twist.linear.x = vx;
	odometryMsg.twist.twist.linear.y = vy;
	odometryMsg.twist.twist.angular.z = deltaTh;

	odometryPublisher.publish(&odometryMsg);

	/////////////////////////

    rangeMsg.range = chassis.range()/100.0;
	rangeMsg.header.frame_id = leftSonarFrameId;
    rangeMsg.header.stamp = nh.now();
    leftRangePublisher.publish(&rangeMsg);

    rangeMsg.range = chassis.range(1)/100.0;
	rangeMsg.header.frame_id = middleSonarFrameId;
    middleRangePublisher.publish(&rangeMsg);

    rangeMsg.range = chassis.range(2)/100.0;
	rangeMsg.header.frame_id = rightSonarFrameId;
    rightRangePublisher.publish(&rangeMsg);

	// middle
	t.header.frame_id = baseFrameId;
	t.child_frame_id = middleSonarFrameId;
	t.transform.translation.x = -0.12;
	t.transform.translation.y = 0;
	t.transform.translation.z = 0;
	
	t.transform.rotation.x = 0; // chassis.orientation().x;
	t.transform.rotation.y = 0; // chassis.orientation().y;
	t.transform.rotation.z = 0; // chassis.orientation().z;
	t.transform.rotation.w = 1; // 0;
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	// left
	t.header.frame_id = baseFrameId;
	t.child_frame_id = leftSonarFrameId;
	t.transform.translation.x = -0.12;
	t.transform.translation.y = -0.06;
	t.transform.translation.z = 0;
	t.transform.rotation = tf::createQuaternionFromYaw(-0.785);
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	// right
	t.header.frame_id = baseFrameId;
	t.child_frame_id = rightSonarFrameId;
	t.transform.translation.x = -0.12;
	t.transform.translation.y = 0.06;
	t.transform.translation.z = 0;
	t.transform.rotation = tf::createQuaternionFromYaw(0.785);
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	// IMU TF
	t.header.frame_id = baseFrameId;
	t.child_frame_id = childFrameId;
	t.transform.translation.x = 0.12;
	t.transform.translation.y = 0;
	t.transform.translation.z = 0.14;
	t.transform.rotation.x = 0; // chassis.orientation().x;
	t.transform.rotation.y = 0; // chassis.orientation().y;
	t.transform.rotation.z = 0; // chassis.orientation().z;
	t.transform.rotation.w = 1; // 0;
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);

	imuMsg.header.stamp = nh.now();
	imuMsg.header.frame_id = baseFrameId;
	// imuMsg.orientation.x = chassis.orientation().x;
	// imuMsg.orientation.y = chassis.orientation().y;
	// imuMsg.orientation.z = chassis.orientation().z;
	imuMsg.orientation = tf::createQuaternionFromYaw(chassis.yaw());
	imuMsg.angular_velocity.x = chassis.gyro().x;
	imuMsg.angular_velocity.y = chassis.gyro().y;
	imuMsg.angular_velocity.z = chassis.gyro().z;
	imuMsg.linear_acceleration.x = chassis.linearAcceleration().x;
	imuMsg.linear_acceleration.y = chassis.linearAcceleration().y;
	imuMsg.linear_acceleration.z = chassis.linearAcceleration().z;
	imuPublisher.publish(&imuMsg);

	long lWheel = 0.5*(chassis.encoderCount(FrontLeft) + chassis.encoderCount(RearLeft));
    long rWheel = 0.5*(chassis.encoderCount(FrontRight) + chassis.encoderCount(RearRight));

    lWheelMsg.data = lWheel;
    rWheelMsg.data = rWheel;
    lWheelPublisher.publish(&lWheelMsg);
    rWheelPublisher.publish(&rWheelMsg);

//     float dt = (now - lastUpdate) / 1E6;
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

//	vLog("L,R: " + String(lWheelTargetRate) + " " + String(rWheelTargetRate));

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
	lastUpdate = now;

 	blinkState = !blinkState;
    digitalWrite(13, blinkState);

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
