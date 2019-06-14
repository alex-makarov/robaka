#include <Arduino.h>

// Pins used on Arduino UNO
// Bluetooth HM-10
const int BLUETOOTH_TX_PIN = 51;
const int BLUETOOTH_RX_PIN = 50;
// Echo sonar HC-SR04
const int TRIGGER_1_PIN = A0; 
const int ECHO_1_PIN    = A1;
const int TRIGGER_2_PIN = A2; 
const int ECHO_2_PIN    = A3;
const int TRIGGER_3_PIN = A4; 
const int ECHO_3_PIN    = A5;
const int TRIGGER_PIN = TRIGGER_1_PIN;
const int ECHO_PIN = ECHO_1_PIN;

// Encoders
const int ENCODER_1_PIN = 46; // TBD
const int ENCODER_4_PIN = 47; // TBD
const int ENCODER_2_PIN = 48; // TBD
const int ENCODER_3_PIN = 49; // TBD
const int ENCODER_RIGHT_PIN = ENCODER_1_PIN;
const int ENCODER_LEFT_PIN = ENCODER_4_PIN;

// TO ADD: ESP8266
