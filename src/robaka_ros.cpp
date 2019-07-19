#include "robaka_ros.h"
#include "chassis.h"
#include "config.h"
#include "utils.h"
#include <tf/tf.h>

#define sign(x) (x >= 0 ? 1 : -1)

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
	: 	odometryPublisher("raw_odom", &odometryMsg),
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
	   	ticksPerMeter(TICKS_PER_METER),
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

	// FIXME when rosserial_arduino >= 0.8.0 is there
	if (_rosnode != NULL) {
		vLog(F("Multiple ROS nodes are not allowed"));
		abort();
	}
	_rosnode = this;

	// Wait until connected
	while (!nh.connected()) { nh.spinOnce(); }
	nh.requestSyncTime();

	// Wait a bit until ROS settles down
	delay(1000);

	if (!nh.getParam("~max_motor_speed", &maxMotorSpeedParam)) {
		nh.logerror("Cannot get max_motor_speed param");
		maxMotorSpeedParam = MAX_MOTOR_SPEED;
	}

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

	this->rosTime = nh.now();
	double timeSinceLastLoop = rosTime.toSec() - lastLoopTs.toSec();

	if (timeSinceLastLoop < -100) {
		// Automatically correct small 'backward time travel' errors
		lastLoopTs = rosTime;
	} else if (timeSinceLastLoop <= 0) {
		// But when it's significant, throw an error
		nh.logerror(String("New timestamp is in the past, skipping loop: " 
					+ String(rosTime.toSec()) + " <= " + String(lastLoopTs.toSec())).c_str());
		return;
	}

	publishSonar();
	publishIMU();
	handleOdometry();
		
	lastUpdate = micros();

	// Blink on every loop
 	blinkState = !blinkState;
    digitalWrite(13, blinkState);

	lastLoopTs = rosTime;
    nh.spinOnce();
}

void RosNode::publishSonar() {
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
}

void RosNode::publishIMU() {
	// Publish IMU
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

	// And magnetic field separately, as it is required by IMU filtering node
	magMsg.header.stamp = rosTime;
	magMsg.header.frame_id = imuFrameId;
	magMsg.magnetic_field.x = chassis.magneticField().x;
	magMsg.magnetic_field.y = chassis.magneticField().y;
	magMsg.magnetic_field.z = chassis.magneticField().z;
	magPublisher.publish(&magMsg);
}

void RosNode::handleOdometry() {
	// Most models are for two-wheel differential drive robots.
	// Since we have four wheels, we just average the encoder counts from the front and rear wheels
	long lWheel = 0.5*(chassis.encoderCount(FrontLeft) + chassis.encoderCount(RearLeft));
    long rWheel = 0.5*(chassis.encoderCount(FrontRight) + chassis.encoderCount(RearRight));

    float dt = (micros() - lastUpdate) / 1E6;
    float lWheelRate = (lWheel - lWheelLast);
    float rWheelRate = (rWheel - rWheelLast);

	// Publish wheel velocities
    lWheelVelocityMsg.data = (lWheelRate / dt) / TICKS_PER_METER;
    rWheelVelocityMsg.data = (rWheelRate / dt) / TICKS_PER_METER;
    lWheelVelocityPublisher.publish(&lWheelVelocityMsg);
    rWheelVelocityPublisher.publish(&rWheelVelocityMsg);

	// Calculating pose updates
	float vx = (lWheelRate + rWheelRate) / (2.0*TICKS_PER_METER);
	float vy = 0;
	float th = chassis.yaw();
	float deltaX = (vx*cos(th) - vy*sin(th));
	float deltaY = (-vx*sin(th) + vy*cos(th));
	float deltaTh = ((rWheelRate - lWheelRate)/TICKS_PER_METER) / WHEEL_SEPARATION; // Don't use gyro, it's too noisy

	x += deltaX;
	y += deltaY;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
	t.header.frame_id = "odom";
	t.child_frame_id = "base_footprint";
	t.transform.translation.x = x; 
	t.transform.translation.y = y;
	t.transform.translation.z = 0;
	t.transform.rotation = odom_quat;	
	t.header.stamp = rosTime;
	// DO NOT SEND THE TRANSFORM! It will be published by imu_filter_madgwick, using smoothed and filtered values
	// broadcaster.sendTransform(t);

	// Publish "odom"
	odometryMsg.header.stamp = rosTime;
	odometryMsg.header.frame_id = "odom";
	odometryMsg.child_frame_id = "base_footprint";
	odometryMsg.pose.pose.position.x = x;
	odometryMsg.pose.pose.position.y = y;
	odometryMsg.pose.pose.position.z = z;
	odometryMsg.pose.pose.orientation = odom_quat;
	odometryMsg.twist.twist.linear.x = vx / dt;
	odometryMsg.twist.twist.linear.y = 0;
	odometryMsg.twist.twist.angular.z = deltaTh / dt;
	odometryPublisher.publish(&odometryMsg);

// Now, motor control
	int leftControl = leftController.getControlValue(lWheelRate/dt, dt);
	int rightControl = rightController.getControlValue(rWheelRate/dt, dt);
#ifdef DEBUG_ODOMETRY
	nh.loginfo(String("CV: " + String(leftControl) + ", " + String(rightControl)).c_str());
#endif
	leftMotorCmd += min(MAX_MOTOR_CMD, leftControl);
	leftMotorCmd = constrain(leftMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
	if (leftMotorCmd > 0) {
		leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
	} else if (leftMotorCmd < 0) {
		leftMotorCmd = min(leftMotorCmd, -MIN_MOTOR_CMD);
	}

	rightMotorCmd += min(MAX_MOTOR_CMD, rightControl);
	rightMotorCmd = constrain(rightMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
	if (rightMotorCmd > 0) {
		rightMotorCmd = max(rightMotorCmd, MIN_MOTOR_CMD);
	} else if (rightMotorCmd < 0) {
		rightMotorCmd = min(rightMotorCmd, -MIN_MOTOR_CMD);
	}

#ifdef DEBUG_ODOMETRY
	nh.loginfo(String("CE: " + String(leftController.getCumulativeError()) + ", " + String(rightController.getCumulativeError())).c_str());
	nh.loginfo(String("CM: " + String(leftMotorCmd) + ", " + String(rightMotorCmd)).c_str());
#endif

	// Coast to a stop if target is zero.
	if (lWheelTargetRate == 0) {
		leftMotorCmd = 0;
	}
	if (rWheelTargetRate == 0) {
		rightMotorCmd = 0;
	}

	// FIXME: turning assistance
	if (leftMotorCmd != rightMotorCmd && sign(leftMotorCmd) == sign(rightMotorCmd)) {
		if (abs(leftMotorCmd) == MIN_MOTOR_CMD) 
			leftMotorCmd = 120*sign(leftMotorCmd);
		if (abs(rightMotorCmd) == MIN_MOTOR_CMD) 
			rightMotorCmd = 120*sign(rightMotorCmd);
	}

	// Inform the core about our motor commands
    lWheelMsg.data = leftMotorCmd;
    rWheelMsg.data = rightMotorCmd;
    lWheelPublisher.publish(&lWheelMsg);
    rWheelPublisher.publish(&rWheelMsg);

	chassis.moveMotor(FrontLeft, leftMotorCmd);
	chassis.moveMotor(RearLeft, leftMotorCmd);
	chassis.moveMotor(FrontRight, rightMotorCmd);
	chassis.moveMotor(RearRight, rightMotorCmd);

	// Save encoder values for the next iteration
    lWheelLast = lWheel;
    rWheelLast = rWheel;
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
	
	const int tickRate = linearSpeed*TICKS_PER_METER;
    const int diffTicks = angularSpeed*WHEEL_SEPARATION*TICKS_PER_METER * 2; // 2 = let's try
	int lSpeed = tickRate - diffTicks;
	int rSpeed = tickRate + diffTicks;

	//	nh.loginfo(String("cmd_vel x" + String(linearSpeed) + " , angularSpeed: " + String(angularSpeed)).c_str());

	if (max(lSpeed, rSpeed) > maxMotorSpeedParam) {
		float factor = maxMotorSpeedParam / float(max(lSpeed, rSpeed));
		lSpeed *= factor;
		rSpeed *= factor;
	}

	// Translate speed into motor range
	lSpeed = lSpeed * MAX_MOTOR_CMD / maxMotorSpeedParam;
	rSpeed = rSpeed * MAX_MOTOR_CMD / maxMotorSpeedParam;

	rWheelTargetRate = rSpeed;
	lWheelTargetRate = lSpeed;

	rightController.setSetPoint(rWheelTargetRate);
	leftController.setSetPoint(lWheelTargetRate);
	lastMotorCmdTime = millis();
}