#include <Arduino.h>
#include <SoftwareSerial.h>

typedef void (*ISR)();

/* 
LIBRARY PATCHES REQUIRED TO MAKE IT WORK ON ARDUINO UNO

1. I2C - modification of Adafruit_L3GD20_U.h to correct the IDs as described:

  !! NOTE that Gyro is using non-standard address 0x69 instead of 0x6B and D3 id instead of D4 or D7
    I2C device found at address 0x69  
    // #define L3GD20_ADDRESS           (0x6B)        // 1101011
    #define L3GD20_ADDRESS           (0x69)        // 1101001
    ...
    // #define L3GD20_ID                (0xD4)
    // #define L3GD20H_ID               (0xD7)
    #define L3GD20_ID                (0xD3)
    #define L3GD20H_ID               (0xD7)

2. MotorDriver library requires header change to redefine the pin used for one motor:
// Note than motor shield pin 3 is remapped to arduino pin 10,
// requiring header change as described here https://samwedge.uk/posts/using-the-arduino-motor-controller-shield-with-two-external-interrupts/

*/

// Pins used on Arduino UNO
// Bluetooth HM-10
const unsigned int BLUETOOTH_TX_PIN = 13;
const unsigned int BLUETOOTH_RX_PIN = A2;
// Echo sonar HC-SR04
const unsigned int TRIGGER_PIN = A1; // Arduino pin tied to trigger pin on the ultrasonic sensor.
const unsigned int ECHO_PIN    = A0; // Arduino pin tied to echo pin on the ultrasonic sensor.

// Encoders
const unsigned int ENCODER_1_PIN = 2; // REAR RIGHT. Interrupt pin.
const unsigned int ENCODER_4_PIN = 3; // REAR LEFT. Interrupt pin.
const unsigned int ENCODER_RIGHT_PIN = ENCODER_1_PIN;
const unsigned int ENCODER_LEFT_PIN = ENCODER_4_PIN;

const unsigned int Encoders[] = {ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN};
volatile unsigned long EncoderUpdates[] = {0,0};
volatile unsigned long EncoderCounts[] = {0,0};
const int N_Encoders = 2;

void rightPinISR() {
    ++EncoderCounts[1];
    EncoderUpdates[1] = millis();
}
void leftPinISR() {
    ++EncoderCounts[0];
    EncoderUpdates[0] = millis();
}

const ISR EncoderISRs[] = {rightPinISR, leftPinISR};

// Rest pins are used for motors: 10 (instead of 3, see above),5,6,11; 4,7,8,12; 
