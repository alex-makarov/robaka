#include <stdarg.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <MotorDriver.h>
#include <NewPing.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

// 
// HW TODO: 
// 1) Swap pin 4 and pin 10
// 2) Maybe use one more sonar instead of two same-axis encoders

///////////////////////////////////////////////////////////////////////////////
// Configuration
#define MOVE_MOTORS 0

const int MAX_SONAR_DISTANCE  = 50; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERIAL_SPEED        = 9600;
const int BLUETOOTH_SPEED     = 9600;
const int CENTER_JOYSTICK     = 49;
const int JOYSTICK_DEADZONE   = 5;

const int MOTOR_REAR_LEFT   = 1;
const int MOTOR_REAR_RIGHT  = 2;
const int MOTOR_FWD_LEFT    = 3;
const int MOTOR_FWD_RIGHT   = 4;

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
const int ENCODER_1_PIN = 10; // used to be 4!
const int ENCODER_2_PIN = 9;
const int ENCODER_3_PIN = A3;
const int ENCODER_4_PIN = A2;
// Rest pins are used for motors: 3,5,6,11; 4,7,8,12; 

///////////////////////////////////////////////////////////////////////////////
// Static objects
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(XMCS_PIN, GCS_PIN);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_SONAR_DISTANCE); // NewPing setup of pins and maximum distance.
SoftwareSerial Bluetooth(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
ArduinoBlue Phone(Bluetooth); // pass reference of Bluetooth object to ArduinoCommander.
MotorDriver m; // see https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield.pdf

///////////////////////////////////////////////////////////////////////////////
// Global variables
const int Encoders[] = {ENCODER_1_PIN, ENCODER_2_PIN, ENCODER_3_PIN, ENCODER_4_PIN};
int EncoderValues[] = {LOW, LOW, LOW, LOW};
const int N_Encoders = 4;

int prevThrottle = CENTER_JOYSTICK;
int prevSteering = CENTER_JOYSTICK;
int prevPing = 0;
int Ping = 0;
int throttle, steering, sliderVal, button, sliderId;

///////////////////////////////////////////////////////////////////////////////
void pf(const char *fmt, ... ){
        char buf[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 128, fmt, args);
        va_end (args);
        Serial.print(buf);
}

void p(const char* string) {
  Serial.println(string);
}

///////////////////////////////////////////////////////////////////////////////
void setupIMU()
{
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    p("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);

  p("Found LSM9DS0 9DOF");
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
  p("Setup complete");
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
      pf("Encoder %d is moving\n", i);
    }

    EncoderValues[i] = val;
  }
}

void readIMU() {
 // IMU
  lsm.read();
  Serial.print("Accel X: ");
  Serial.print((int)lsm.accelData.x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print((int)lsm.accelData.y);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println((int)lsm.accelData.z);
  Serial.print(" ");
  Serial.print("Mag X: ");
  Serial.print((int)lsm.magData.x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print((int)lsm.magData.y);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println((int)lsm.magData.z);
  Serial.print(" ");
  Serial.print("Gyro X: ");
  Serial.print((int)lsm.gyroData.x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print((int)lsm.gyroData.y);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.println((int)lsm.gyroData.z);
  Serial.println(" ");
  Serial.print("Temp: ");
  Serial.print((int)lsm.temperature);
  Serial.println(" ");
}

void updateMotors() {
 // Display throttle and steering data if steering or throttle value is changed
  if (prevThrottle != throttle || prevSteering != steering || prevPing != Ping)
  {
    prevThrottle = throttle;
    prevSteering = steering;

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
}

void readSonars() {
    Ping = sonar.ping_cm();
    pf("[SONAR] %d\n", Ping);
}

// Put your main code here, to run repeatedly:
void loop()
{
  readPhone();
  sendSerialToPhone();

  readSonars();
  readEncoders();
  readIMU();

  updateMotors();  
}
