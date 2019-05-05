#include <stdarg.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

// 
// HW TODO: 
// 2) Maybe use one more sonar instead of two same-axis encoders

// I2C conf
// I2C device found at address 0x19  !
// I2C device found at address 0x1E  !
// I2C device found at address 0x69  ! ! NOTE that Gyro is using non-standard address 0x69 instead of 0x6B
//  and D3 id instead of D4 or D7
//    #define L3GD20_ADDRESS           (0x6B)        // 1101011
    // #define L3GD20_ADDRESS           (0x69)        // 1101011
    // #define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
    // // #define L3GD20_ID                (0xD4)
    // // #define L3GD20H_ID               (0xD7)
    // #define L3GD20_ID                (0xD3)
    // #define L3GD20H_ID               (0xD7)

///////////////////////////////////////////////////////////////////////////////
// Configuration
#define MOVE_MOTORS 1

#undef IMU_DEBUG
//#define IMU_DEBUG

const int MAX_SONAR_DISTANCE  = 50; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERIAL_SPEED        = 9600;
const int BLUETOOTH_SPEED     = 9600;
const int CENTER_JOYSTICK     = 49;
const int JOYSTICK_DEADZONE   = 5;

//1423
const int MOTOR_REAR_LEFT   = 1;
const int MOTOR_REAR_RIGHT  = 4;
const int MOTOR_FWD_LEFT    = 2;
const int MOTOR_FWD_RIGHT   = 3;

const int BUTTON_STOP = 0;
const int BUTTON_START = 1;

const int PROXIMITY_THRESHOLD = 15; // cm
const int TURN_DURATION = 2000; // us
const int BACKWARD_DURATION = 2000; // us
const int STUCK_THRESHOLD = 500; // us
const int DEFAULT_MOTOR_SPEED = 200; // x/255

// Pins
// Bluetooth
const int BLUETOOTH_TX_PIN = 13;
const int BLUETOOTH_RX_PIN = 2;
// Echo sonar
const int TRIGGER_PIN = A1; // Arduino pin tied to trigger pin on the ultrasonic sensor.
const int ECHO_PIN    = A0; // Arduino pin tied to echo pin on the ultrasonic sensor.
// IMU
const int XMCS_PIN  = A5;
const int GCS_PIN   = A4;
// Encoders
const int ENCODER_1_PIN = 10; // REAR RIGHT
const int ENCODER_2_PIN = 9;  // FRONT RIGHT
const int ENCODER_3_PIN = A3; // FRONT LEFT
const int ENCODER_4_PIN = A2; // REAR LEFT
// Rest pins are used for motors: 3,5,6,11; 4,7,8,12; 

///////////////////////////////////////////////////////////////////////////////
// Static objects
Adafruit_9DOF imu = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE); // NewPing setup of pins and maximum distance.
SoftwareSerial Bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
ArduinoBlue Phone(Bluetooth); // pass reference of Bluetooth object to ArduinoCommander.
MotorDriver m; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

///////////////////////////////////////////////////////////////////////////////
// Global variables
const int Encoders[] = {ENCODER_1_PIN, ENCODER_2_PIN, ENCODER_3_PIN, ENCODER_4_PIN};
int EncoderValues[] = {LOW, LOW, LOW, LOW};
unsigned long EncoderUpdates[] = {0,0,0,0};
const int N_Encoders = 4;

int prevThrottle = CENTER_JOYSTICK;
int prevSteering = CENTER_JOYSTICK;
int prevPing = 0;
int Ping = 0;
int throttle, steering, sliderVal, button, sliderId;

unsigned long moveStarted = 0;

enum State {
  Stop = 0,
  Forward,
  Backward,
  RightTurn,
  LeftTurn
} state;

const char *PrintableStates[] = {
  "STOP",
  "FORWARD",
  "BACKWARD",
  "RIGHT",
  "LEFT"
};

///////////////////////////////////////////////////////////////////////////////
void pf(const char *fmt, ... ){
        char buf[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 256, fmt, args);
        va_end (args);
        Serial.print(buf);
}

void p(const char* string) {
  Serial.println(string);
}

void pphone(const char *fmt, ... ) {
        char buf[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 256, fmt, args);
        va_end (args);

        Phone.sendMessage(String(buf));
}

///////////////////////////////////////////////////////////////////////////////
void setupIMU()
{
//gyro.enableAutoRange(true);
  if (!gyro.begin()) {
    p("Oops ... unable to initialize the Gyroscope. Check your wiring!");
    while (1);
  }


  // Try to initialise and warn if we couldn't detect the chip
  if (!imu.begin() || !accel.begin() || !mag.begin()) { 
    p("Oops ... unable to initialize the IMU. Check your wiring!");
    while (1);
  }
  
  
  p("Found LSM303DLHC and L3GD20 IMU");
}

///////////////////////////////////////////////////////////////////////////////
// Setup code runs once after program starts.
void setup()
{
  Serial.begin(SERIAL_SPEED);
  p("Hello");

  Bluetooth.begin(BLUETOOTH_SPEED);

  // Init pins for encoders
  for (int i = 0; i < N_Encoders; i++)
  {
    pinMode(Encoders[i], INPUT);
  }

  setupIMU();

  state = Stop;

  p("Setup complete");
  pphone("READY");
}

void readPhone()
{
  // ID of the button pressed pressed.
  button = Phone.getButton();

  // Returns the text data sent from the Phone.
  // After it returns the latest data, empty string "" is sent in subsequent.
  // calls until text data is sent again.
  String str = Phone.getText();
  if (str != "")
  {
    p(str.c_str());
  }

  // Throttle and steering values go from 0 to 99.
  // When throttle and steering values are at 99/2 = 49, the joystick is at center.
  throttle = Phone.getThrottle();
  steering = Phone.getSteering();

  // ID of the slider moved.
  sliderId = Phone.getSliderId();

  // Slider value goes from 0 to 200.
  sliderVal = Phone.getSliderVal();

  // Display button data whenever its pressed.
  if (button != -1)
  {
    pf("Button %d\n", button);
  }

  // Display slider data when slider moves
  if (sliderId != -1)
  {
    pf("Slider[%d]: %d\n", sliderId, sliderVal);
  }
}

void sendSerialToPhone() {
  // Send string from serial command line to the Phone. This will alert the user.
  if (Serial.available())
  {    
    Serial.write("send: ");
    String str = Serial.readString();
    Phone.sendMessage(str); // Phone.sendMessage(str) sends the text to the Phone.
    Serial.print(str);
    Serial.write('\n');
  }
}

void readEncoders() {
  for (int i = 0; i < N_Encoders; i++)
  {
    int val = digitalRead(Encoders[i]);

    if (val != EncoderValues[i])
    {
      pf("%d is moving\n", i);
      EncoderUpdates[i] = millis();
    }

    EncoderValues[i] = val;
  }
}

void readIMU() {
 // IMU
sensors_event_t event;
sensors_vec_t orientation;
//sensors_axis_t axis;

#ifdef IMU_DEBUG
pf ("IMU [roll, pitch, heading, x/y/z] [");
#endif

accel.getEvent(&event);
if (imu.accelGetOrientation(&event, &orientation)) {
  #ifdef IMU_DEBUG
  Serial.print(orientation.roll);
  Serial.print(", ");
  Serial.print(orientation.pitch);
  Serial.print(", ");
  #endif
}

mag.getEvent(&event);
if (imu.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation)) {
  #ifdef IMU_DEBUG
  Serial.print(orientation.heading);
  Serial.print(", ");
  #endif
}

gyro.getEvent(&event);
#ifdef IMU_DEBUG
Serial.print(event.gyro.x);
Serial.print("/");
Serial.print(event.gyro.y);
Serial.print("/");
Serial.print(event.gyro.z);
pf("]\n");
#endif
}

void updateMotors() {
#if 0  
 // Display throttle and steering data if steering or throttle value is changed
  if (prevThrottle != throttle || prevSteering != steering || prevPing != Ping)
  {
    prevThrottle = throttle;
    prevSteering = steering;
    prevPing = Ping;

    int corrected = 0;
    if (throttle > CENTER_JOYSTICK)
    {
      corrected = (throttle - CENTER_JOYSTICK) * 5;
    }
    else if (throttle < CENTER_JOYSTICK)
    {
      corrected = (CENTER_JOYSTICK - throttle) * 5;
    }

    int direction = throttle > CENTER_JOYSTICK ? BACKWARD : FORWARD;
    if ((Ping >= 1 && Ping <= 15 && direction == BACKWARD) || throttle == CENTER_JOYSTICK)
    {
      corrected = 0;
    }

    pf("Throttle %d, steering %d, corrected %d, direction %d\n", throttle, steering, corrected, direction);

#ifdef MOVE_MOTORS
    if (steering > CENTER_JOYSTICK + JOYSTICK_DEADZONE)
    {
      m.motor(MOTOR_FWD_LEFT, direction, corrected);
      m.motor(MOTOR_REAR_LEFT, direction, corrected);
      m.motor(MOTOR_FWD_RIGHT, direction, 0);
      m.motor(MOTOR_REAR_RIGHT, direction, 0);
    }
    else if (steering < CENTER_JOYSTICK - JOYSTICK_DEADZONE)
    {
      m.motor(MOTOR_FWD_RIGHT, direction, corrected);
      m.motor(MOTOR_REAR_RIGHT, direction, corrected);
      m.motor(MOTOR_FWD_LEFT, direction, 0);
      m.motor(MOTOR_REAR_LEFT, direction, 0);
    }
    else
    {
      m.motor(MOTOR_FWD_RIGHT, direction, corrected);
      m.motor(MOTOR_REAR_RIGHT, direction, corrected);
      m.motor(MOTOR_FWD_LEFT, direction, corrected);
      m.motor(MOTOR_REAR_LEFT, direction, corrected);
    }
#endif
  }
#endif // 0
}

void readSonars() {
    Ping = sonar.ping_cm();
    if (Ping != 0) {
//      pf("[SONAR] %d\n", Ping);
    }
}

// Put your main code here, to run repeatedly:
void loop()
{
  readPhone();
  sendSerialToPhone();

  readSonars();
  readEncoders();
  readIMU();

//  updateMotors();  
  step();
  updateMotorsOnStep();
}

void step() {

  State prevState = state;

  switch (state) {
    case Forward:
      if (Ping < PROXIMITY_THRESHOLD && Ping > 0) {
        pf("Ping %d", Ping);
        state = Backward;
        moveStarted = millis();
      }

      if (isStuck()) {
        state = Backward;
        moveStarted = millis();
      }

      break;

    case Backward:
      if (millis() - moveStarted >= BACKWARD_DURATION) {
        state = random(0,2) > 0 ? RightTurn : LeftTurn;
        moveStarted = millis();
      }
      break;

    case RightTurn:
    case LeftTurn:
      if (millis() - moveStarted >= TURN_DURATION) {
        state = Forward;
        moveStarted = millis();
      }
      break;

    case Stop:
      if (button == BUTTON_START) {
        state = Forward;
        moveStarted = millis();
        for (int i = 0; i < N_Encoders; i++) {
          EncoderUpdates[i] = millis();
        }
      }

      break;

    default: 
      break;
  }

  if (button == BUTTON_STOP) {
    state = Stop;
    moveStarted = millis();
  }

  if (prevState != state) {
    pf("State: %s -> %s\n", PrintableStates[prevState], PrintableStates[state]);
    pphone("State: %s -> %s", PrintableStates[prevState], PrintableStates[state]);
  }
}

void updateMotorsOnStep() {
#if (MOVE_MOTORS == 0) 
  return;
#endif

  int speed = DEFAULT_MOTOR_SPEED;

  switch (state) {
    case Forward:
      moveMotor(MOTOR_FWD_LEFT, FORWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, FORWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, FORWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, FORWARD, speed);
      break;

    case Backward:
      moveMotor(MOTOR_FWD_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, BACKWARD, speed);
      break;

    case RightTurn:
      moveMotor(MOTOR_FWD_LEFT, FORWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, FORWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, BACKWARD, speed);
      break;

    case LeftTurn:
      moveMotor(MOTOR_FWD_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_REAR_LEFT, BACKWARD, speed);
      moveMotor(MOTOR_FWD_RIGHT, FORWARD, speed);
      moveMotor(MOTOR_REAR_RIGHT, FORWARD, speed);
      break;

    case Stop:
      moveMotor(MOTOR_FWD_LEFT, FORWARD, 0);
      moveMotor(MOTOR_REAR_LEFT, FORWARD, 0);
      moveMotor(MOTOR_FWD_RIGHT, FORWARD, 0);
      moveMotor(MOTOR_REAR_RIGHT, FORWARD, 0);
      break;

    default:
    break;
  }
}

void moveMotor(int motorId, int direction, int speed) {
  switch(motorId) {
    case MOTOR_REAR_RIGHT:
    case MOTOR_FWD_RIGHT:
      m.motor(motorId, direction == FORWARD ? BACKWARD : FORWARD, speed);
      break;
    default:
      m.motor(motorId, direction, speed);
      break;
  }
}

bool isStuck() {
  unsigned long now = millis();
  int wheelsStuck = 0;

  for (int i = 0; i < N_Encoders; i++) {
    if (now - EncoderUpdates[i] >= STUCK_THRESHOLD)
      ++wheelsStuck;
  }  

  return wheelsStuck > 3;
}