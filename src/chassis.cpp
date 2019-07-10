#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>
#include <SimplePID.h>
#include <DueTimer.h>

#include "chassis.h"
#include "config.h"
#include "utils.h"

#ifdef ARDUINO_AVR_UNO
#include "hw_uno.h"
#elif ARDUINO_SAM_DUE
#include "hw_due.h"
#endif

class Chassis::HWImpl {
public:
    Adafruit_9DOF imu = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

    NewPing *sonars[N_Sonars];
    MotorDriver motor; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

    sensors_event_t accelEvent;
    sensors_event_t magEvent;
    sensors_event_t gyroEvent;
    sensors_vec_t orientation;

    volatile int pings[N_Sonars];

    bool initIMU();
    bool initSonar();
    bool initEncoders();

    void readIMU();
    void readSonar();

    void attachInterrupts();
    void detachInterrupts();

	void timerCallback();

	volatile long lastEncoderValues[N_Encoders]; // updated from the interrupt
    volatile float wheelSpeeds[N_Encoders];
	volatile unsigned long lastSpeedTimer;
};

bool Chassis::HWImpl::initIMU() {
	if (!gyro.begin(GYRO_RANGE_250DPS, &Wire1)) {
		vLog("Oops ... unable to initialize the Gyroscope. Check your wiring!");
		return false;
	}
	vLog(F("[OK] Gyro init"));

	// Try to initialise and warn if we couldn't detect the chip
	if (!imu.begin() || !accel.begin() || !mag.begin()) { 
		vLog("Oops ... unable to initialize the IMU. Check your wiring!");
		return false;
	}
	vLog(F("[OK] IMU init"));
    
	vLog("[OK] Found LSM303DLHC and L3GD20 IMU");
	return true;
}

bool Chassis::HWImpl::initSonar() {
	for (int i = 0; i < N_Sonars; i++) {
		sonars[i] = new NewPing(SonarPins[i*2], SonarPins[i*2+1], MAX_SONAR_DISTANCE);
	}
	vLog(F("[OK] Sonar init"));
	return true;
}

bool Chassis::HWImpl::initEncoders() {
	for (int i = 0; i < N_Encoders; i++) {
		pinMode(Encoders[i], INPUT);
	}
	lastSpeedTimer = millis();
	attachInterrupts();
	vLog(F("[OK] Encoders init"));
	return true;
}

void Chassis::HWImpl::attachInterrupts() {
	for (int i = 0; i < N_Encoders; i++) {
		attachInterrupt(digitalPinToInterrupt(Encoders[i]), EncoderISRs[i], CHANGE);
		pinMode(Encoders[i], INPUT);
	}
}

void Chassis::HWImpl::detachInterrupts() {
	for (int i = 0; i < N_Encoders; i++) {
		detachInterrupt(digitalPinToInterrupt(Encoders[i]));
	}
}

void Chassis::HWImpl::readIMU() {
	sensors_event_t _accelEvent, _magEvent, _gyroEvent;
	sensors_vec_t _orientation;

	accel.getEvent(&_accelEvent);
	mag.getEvent(&_magEvent);
	gyro.getEvent(&_gyroEvent);

	bool success = imu.fusionGetOrientation(&_accelEvent, &_magEvent, &_orientation);
	if (success) {
		accelEvent = _accelEvent;
		magEvent = _magEvent;
		gyroEvent =  _gyroEvent;
		orientation = _orientation;
#ifdef IMU_DEBUG
		vLog("[IMU] R,P,H,X,Y,Z = " +
			 String(orientation.roll) + ", " +
			 String(orientation.pitch) + ", " +
			 String(orientation.heading) + " ," +
			 String(orientation.x) + ", " +
			 String(orientation.y) + ", " +
			 String(orientation.z));
	 	vLog("[Gyro] x,y,z = " + 
		 String(gyroEvent.gyro.x) + ", " +
		 String(gyroEvent.gyro.y) + ", " +
		 String(gyroEvent.gyro.z));
#endif
	}
#ifdef IMU_DEBUG
	else {
		vLog("IMU error");
	}
#endif
}

void Chassis::HWImpl::readSonar() {
#ifdef SONAR_DEBUG
	String s;
	for (int i = 0; i < N_Sonars; i++) {
		pings[i] = sonars[i]->ping_cm();
		s += String(pings[i]) + " ";
	}
	vLog("[Sonar] " + s + "cm");
#else
	for (int i = 0; i < N_Sonars; i++) {
		pings[i] = sonars[i]->ping_cm();
	}
#endif
}

void Chassis::HWImpl::timerCallback() {
	//vLog("timer callback");
	unsigned long dt = (millis() - lastSpeedTimer) / 1E3;
	for (int i = 0; i < N_Encoders; i++) {
		wheelSpeeds[i] = dt * (EncoderCounts[i] - lastEncoderValues[i]) / TICKS_PER_METER;
		//vLog("Speed " + String(i) + " = " + String(wheelSpeeds[i]));
		lastEncoderValues[i] = EncoderCounts[i];
	}
	lastSpeedTimer = millis();
}

///////////////////////////////////////////////////////////////////////////////

Chassis* Chassis::_instance = 0;

Chassis* Chassis::instance() {
    if (_instance == 0) {
        _instance = new Chassis();
		Timer.getAvailable().attachInterrupt(timerCallback).setFrequency(1).start(1E6);
    }

    return _instance;
}

Chassis::Chassis() {
    impl = new Chassis::HWImpl();
    lastUpdate = 0;
}

Chassis::~Chassis() {
    delete impl;
}

void Chassis::timerCallback() {
	Chassis::instance()->impl->timerCallback();
}

bool Chassis::init() {
    initialized =  impl->initEncoders() &&
        impl->initSonar() &&
        impl->initIMU();
    return initialized;
}

void Chassis::updateSensors() {
    impl->detachInterrupts();
    impl->readIMU();
    impl->readSonar();
    impl->attachInterrupts();
    lastUpdate = millis();
}

void Chassis::moveMotor (Wheel wheel, int speed) {
	moveMotor(wheel, speed >= 0 ? Forward : Backward, speed);
}

void Chassis::moveMotor (Wheel wheel, Direction direction, int speed) {

    int motorId = 0;
    switch (wheel) {
	case FrontLeft: motorId = MOTOR_FWD_LEFT; break;
	case FrontRight: motorId = MOTOR_FWD_RIGHT; break;
	case RearLeft: motorId = MOTOR_REAR_LEFT; break;
	case RearRight: motorId = MOTOR_REAR_RIGHT; break;
    }
    int _direction = direction == Forward ? FORWARD : BACKWARD;

	EncoderDirections[wheelToEncoder(wheel)] = (_direction == FORWARD) ? 1 : -1;

	if (speed == 0) {
		_direction = RELEASE;
	}
	int _speed = abs(speed);

	#ifdef MOTOR_DEBUG
	vLog("M" + String(motorId) + " <" + String(direction) + "> := " + String(speed));
	#endif

	// Mapping according to motor orientation in the chassis
#ifdef MOVE_MOTORS
	switch(motorId) {
    case MOTOR_REAR_RIGHT:
    case MOTOR_FWD_RIGHT:
		if (_direction == FORWARD || _direction == BACKWARD) {
			// Only remap motor direction for rotation, not for braking or coasting
			impl->motor.motor(motorId, _direction == FORWARD ? BACKWARD : FORWARD, _speed);
		} else {
			impl->motor.motor(motorId, _direction, _speed);
		}
		break;
    default:
		impl->motor.motor(motorId, _direction, _speed);
		break;
	}
#endif
}

int Chassis::heading() const {
	return(int)impl->orientation.heading;
}

float Chassis:: yaw() const {
	return (heading() - 90) * PI / 180.0; 
}

float Chassis::roll() const {
    return impl->orientation.roll;
}

float Chassis::pitch() const {
    return impl->orientation.pitch;
}

vector_t Chassis::gyro() const {
	return { impl->gyroEvent.gyro.x, impl->gyroEvent.gyro.y, impl->gyroEvent.gyro.z };
}

vector_t Chassis::orientation() const {
	return { impl->orientation.x, impl->orientation.y, impl->orientation.z };
}

vector_t Chassis::linearAcceleration() const {
	return { impl->accelEvent.acceleration.x, impl->accelEvent.acceleration.y, impl->accelEvent.acceleration.z };
}

float Chassis::speedMs() const {
	float s = 0.0;
	for (int i = 0; i < N_Encoders; i++)
		s += impl->wheelSpeeds[i];
	return s / N_Encoders;
}

int Chassis::range(const int sonar) const {
    if (sonar < 0 || sonar > N_Sonars-1) 
        return -1;

    return impl->pings[sonar];
}

long Chassis::encoderCount(Wheel wheel) const {
    int encoder = wheelToEncoder(wheel);

    if (encoder < 0 || encoder > N_Encoders-1)
        return -1;

    return EncoderCounts[encoder];
}

unsigned long Chassis::lastUpdateTs() const {
    return lastUpdate;
}

int Chassis::wheelToEncoder(Wheel wheel) const {
    switch (wheel) {
	case FrontLeft: return 1;
	case FrontRight: return 2;
	case RearLeft: return 0;
	case RearRight: return 3;
    }
    return 0;
}
