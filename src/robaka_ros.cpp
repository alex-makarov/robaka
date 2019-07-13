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

void _cmdvelCallback(const geometry_msgs::Twist& cmdMsg) {
	if (_rosnode) {
		_rosnode->cmdvelCallback(cmdMsg);
	}
}

RosNode :: RosNode(Chassis& _chassis)
	:  	ticksPerMeter(TICKS_PER_METER),
		odometryPublisher("raw_odom", &odometryMsg),
	   	leftRangePublisher("sonar_left", &rangeMsg),
		middleRangePublisher("sonar_middle", &rangeMsg),
		rightRangePublisher("sonar_right", &rangeMsg),	   	   
	   lWheelPublisher("lwheel", &lWheelMsg),
	   rWheelPublisher("rwheel", &rWheelMsg),
	   imuPublisher("imu/data_raw", &imuMsg),
	   magPublisher("imu/mag", &magMsg),
	   lWheelVelocityPublisher("lwheel_velocity", &lWheelVelocityMsg),
	   rWheelVelocityPublisher("rwheel_velocity", &rWheelVelocityMsg),
	   lWheelTargetSub("lwheel_vtarget", &lWheelTargetCallback),
	   rWheelTargetSub("rwheel_vtarget", &rWheelTargetCallback),
	   cmdvelSub("cmd_vel", &_cmdvelCallback),
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
	nh.advertise(magPublisher);

	nh.subscribe(lWheelTargetSub);
	nh.subscribe(rWheelTargetSub);
	nh.subscribe(cmdvelSub);

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

	nh.spinOnce();
	nh.requestSyncTime();

	delay(1000);

	lastLoopTs = ros::Time();
	lastUpdate = micros();
	lastMotorCmdTime = millis();
}

void RosNode::loop() {
	if (!nh.connected()) {
		nh.logerror("ROS not connected");
		nh.spinOnce();
		return;
	}

    unsigned long now = micros();
	ros::Time rosTime = nh.now();
	double timeSinceLastLoop = rosTime.toSec() - lastLoopTs.toSec();

	if (timeSinceLastLoop < -100) {
		lastLoopTs = rosTime;
	} else if (timeSinceLastLoop <= 0) {
		nh.logerror(String("New timestamp is in the past, skipping loop: " 
					+ String(rosTime.toSec()) + " <= " + String(lastLoopTs.toSec())).c_str());
		return;
	}

	// See http://www.ros.org/reps/rep-0103.html for coordinate references
    rangeMsg.range = chassis.range(0)/100.0;
	rangeMsg.header.frame_id = leftSonarFrameId;
    rangeMsg.header.stamp = rosTime;
    leftRangePublisher.publish(&rangeMsg);

    rangeMsg.range = chassis.range(1)/100.0;
	rangeMsg.header.frame_id = middleSonarFrameId;
    middleRangePublisher.publish(&rangeMsg);

    rangeMsg.range = chassis.range(2)/100.0;
	rangeMsg.header.frame_id = rightSonarFrameId;
    rightRangePublisher.publish(&rangeMsg);

	imuMsg.header.stamp = rosTime;
	imuMsg.header.frame_id = imuFrameId;
	imuMsg.orientation = tf::createQuaternionFromYaw(chassis.yaw());
	imuMsg.angular_velocity.x = chassis.gyro().x;
	imuMsg.angular_velocity.y = chassis.gyro().y;
	imuMsg.angular_velocity.z = chassis.gyro().z;
	imuMsg.linear_acceleration.x = chassis.linearAcceleration().x;
	imuMsg.linear_acceleration.y = chassis.linearAcceleration().y;
	imuMsg.linear_acceleration.z = chassis.linearAcceleration().z;
	imuPublisher.publish(&imuMsg);

	magMsg.header.stamp = rosTime;
	magMsg.header.frame_id = imuFrameId;
	magMsg.magnetic_field.x = chassis.magneticField().x;
	magMsg.magnetic_field.y = chassis.magneticField().y;
	magMsg.magnetic_field.z = chassis.magneticField().z;
	magPublisher.publish(&magMsg);

	// nh.logdebug(">>>>>");
	// nh.loginfo(String("Left encoder counts: " + String(chassis.encoderCount(FrontLeft)) + " " + String(chassis.encoderCount(RearLeft))).c_str());
	// nh.loginfo(String("Right encoder counts: " + String(chassis.encoderCount(FrontRight)) + " " + String(chassis.encoderCount(RearRight))).c_str());

	long lWheel = 0.5*(chassis.encoderCount(FrontLeft) + chassis.encoderCount(RearLeft));
    long rWheel = 0.5*(chassis.encoderCount(FrontRight) + chassis.encoderCount(RearRight));

	// nh.loginfo(String("lWheel " + String(lWheel) + " , rWheel " + String(rWheel)).c_str());

    lWheelMsg.data = lWheel;
    rWheelMsg.data = rWheel;
    lWheelPublisher.publish(&lWheelMsg);
    rWheelPublisher.publish(&rWheelMsg);

    float dt = (now - lastUpdate) / 1E6;
    float lWheelRate = (lWheel - lWheelLast); // / dt;
    float rWheelRate = (rWheel - rWheelLast); // / dt;

    lWheelVelocityMsg.data = (lWheelRate / dt) / TICKS_PER_METER;
    rWheelVelocityMsg.data = (rWheelRate / dt) / TICKS_PER_METER;
    lWheelVelocityPublisher.publish(&lWheelVelocityMsg);
    rWheelVelocityPublisher.publish(&rWheelVelocityMsg);

	// ODOMETRY /////
	float vx = (lWheelRate + rWheelRate) / (2.0*TICKS_PER_METER);         //chassis.speedMs();
	float vy = 0;
	float th = chassis.yaw();
	float deltaX = (vx*cos(th) - vy*sin(th)); // * dt;
	float deltaY = (-vx*sin(th) + vy*cos(th)); // * dt;
	float deltaTh = ((rWheelRate - lWheelRate)/TICKS_PER_METER) / 0.13;
	//chassis.gyro().z;

	x += deltaX;
	y += deltaY;

	// nh.loginfo(String("lWheelVelocity " + String(lWheelVelocityMsg.data) + " , rWheelVelocity " + String(rWheelVelocityMsg.data)).c_str());

	// nh.loginfo(String(
	// 	"lr " + String(lWheelRate) + " "
	// 	"rr " + String(rWheelRate) + " "
	// 	"dt " + String(dt) + " "
	// 	"vx " + String(vx) + " " +
	// 	"th " + String(th) + " " +
	// 	"dx " + String(deltaX) + " " + 
	// 	"dy " + String(deltaY) + " " +
	// 	"dTh " + String(deltaTh) + " " +
	// 	"x " + String(x) + " " +
	// 	"y " + String(y) + " " +
	// 	"gyroZ " + String(chassis.gyro().z)
	// ).c_str());

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
	t.header.frame_id = "odom";
	t.child_frame_id = "base_footprint";
	t.transform.translation.x = x; 
	t.transform.translation.y = y;
	t.transform.translation.z = 0;
	t.transform.rotation = odom_quat;	
	t.header.stamp = rosTime;
//	broadcaster.sendTransform(t);

	odometryMsg.header.stamp = rosTime;
	odometryMsg.header.frame_id = "odom";
	odometryMsg.pose.pose.position.x = x;
	odometryMsg.pose.pose.position.y = y;
	odometryMsg.pose.pose.position.z = z;
	odometryMsg.pose.pose.orientation = odom_quat;

	odometryMsg.child_frame_id = "base_footprint";
	odometryMsg.twist.twist.linear.x = vx / dt;
	odometryMsg.twist.twist.linear.y = 0;
	odometryMsg.twist.twist.angular.z = deltaTh / dt;

	odometryPublisher.publish(&odometryMsg);


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

	// hack, FIXME
	if (leftMotorCmd >= 198) {
		leftMotorCmd = 255;
	}
	if (rightMotorCmd >= 198) {
		rightMotorCmd = 255;
	}

//	nh.loginfo(String("Actual commands: " + String(leftMotorCmd) + ", Right: " + String(rightMotorCmd)).c_str());

	chassis.moveMotor(FrontLeft, leftMotorCmd);
	chassis.moveMotor(RearLeft, leftMotorCmd);
	chassis.moveMotor(FrontRight, rightMotorCmd);
	chassis.moveMotor(RearRight, rightMotorCmd);

    lWheelLast = lWheel;
    rWheelLast = rWheel;
	lastUpdate = now;

 	blinkState = !blinkState;
    digitalWrite(13, blinkState);

	lastLoopTs = rosTime;

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

void RosNode::cmdvelCallback(const geometry_msgs::Twist& cmdMsg) {

	const float linearSpeed = cmdMsg.linear.x;
	const float angularSpeed = cmdMsg.angular.z;

//	nh.loginfo(String("cmd_vel x" + String(linearSpeed) + " , angularSpeed: " + String(angularSpeed)).c_str());

	const int ticksPerMeter = TICKS_PER_METER;
	const float wheelSeparation = 0.13; // meters between wheels
	const float maxMotorSpeed = 198; // ticks per second (=1m)

	const float tickRate = linearSpeed*ticksPerMeter;
    const int diffTicks = angularSpeed*wheelSeparation*ticksPerMeter;

	int lSpeed = tickRate - diffTicks;
	int rSpeed = tickRate + diffTicks;

	if (max(lSpeed, rSpeed) > maxMotorSpeed) {
		float factor = maxMotorSpeed / max(lSpeed, rSpeed);
		lSpeed *= factor;
		rSpeed *= factor;
	}

//	nh.loginfo(String("Left: " + String(lSpeed) + ", Right: " + String(rSpeed)).c_str());
	rWheelTargetRate = rSpeed;
	lWheelTargetRate = lSpeed;
	rightController.setSetPoint(rWheelTargetRate);
	leftController.setSetPoint(lWheelTargetRate);
	lastMotorCmdTime = millis();
}