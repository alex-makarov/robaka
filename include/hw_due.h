#include <Arduino.h>

typedef void (*ISR)();

// Pins used on Arduino UNO
// Bluetooth HM-10
const unsigned int BLUETOOTH_TX_PIN = 51;
const unsigned int BLUETOOTH_RX_PIN = 50;
// Echo sonar HC-SR04
const unsigned int TRIGGER_1_PIN = A0; 
const unsigned int ECHO_1_PIN    = A1;
const unsigned int TRIGGER_2_PIN = A2; 
const unsigned int ECHO_2_PIN    = A3;
const unsigned int TRIGGER_3_PIN = A4; 
const unsigned int ECHO_3_PIN    = A5;
const int N_Sonars = 3;
const unsigned int SonarPins[] = { TRIGGER_1_PIN, ECHO_1_PIN,
                                TRIGGER_2_PIN, ECHO_2_PIN,
                                TRIGGER_3_PIN, ECHO_3_PIN };

// Encoders
const unsigned int ENCODER_1_PIN = 46; // TBD
const unsigned int ENCODER_4_PIN = 47; // TBD
const unsigned int ENCODER_2_PIN = 48; // TBD
const unsigned int ENCODER_3_PIN = 49; // TBD

const unsigned int Encoders[] = {ENCODER_1_PIN, ENCODER_2_PIN, ENCODER_3_PIN, ENCODER_4_PIN};
volatile unsigned long EncoderUpdates[] = {0,0,0,0};
volatile unsigned long EncoderCounts[] = {0,0,0,0};
const int N_Encoders = 4;

void EncoderISR0() {
    ++EncoderCounts[0];
    EncoderUpdates[0] = millis();
}
void EncoderISR1() {
    ++EncoderCounts[1];
    EncoderUpdates[1] = millis();
}
void EncoderISR2() {
    ++EncoderCounts[2];
    EncoderUpdates[2] = millis();
}
void EncoderISR3() {
    ++EncoderCounts[3];
    EncoderUpdates[3] = millis();
}

const ISR EncoderISRs[] = {EncoderISR0, EncoderISR1, EncoderISR2, EncoderISR3};


// TO ADD: ESP8266
