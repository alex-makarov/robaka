#include <Arduino.h>
#include "utils.h"
#include "config.h"

#ifndef _HW_DUE_H
#define _HW_DUE_H

typedef void (*ISR)();

// Pins used on Arduino DUE

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
const unsigned int SONAR_LEFT = 0;
const unsigned int SONAR_MIDDLE = 1;
const unsigned int SONAR_RIGHT = 2;

// Encoders
const unsigned int ENCODER_1_PIN = 46; // LEFT REAR
const unsigned int ENCODER_4_PIN = 47; // RIGHT REAR
const unsigned int ENCODER_2_PIN = 48; // LEFT FRONT
const unsigned int ENCODER_3_PIN = 49; // RIGHT FRONT

const unsigned int Encoders[] = {ENCODER_1_PIN, ENCODER_2_PIN, ENCODER_3_PIN, ENCODER_4_PIN};
volatile unsigned long EncoderUpdates[] = {0,0,0,0};
volatile long EncoderCounts[] = {0,0,0,0};
int EncoderDirections[] = {1,1,1,1};
const int N_Encoders = 4;

void EncoderISR0() {
    EncoderCounts[0] += EncoderDirections[0];
    EncoderUpdates[0] = millis();
#ifdef ENCODER_DEBUG
// Normally logging in an interrupt is a 'no-no', but this is only for encoder troubleshooting cases.
    vLog("Encoder 0" + String(EncoderCounts[0]));
#endif
}
void EncoderISR1() {
    EncoderCounts[1] += EncoderDirections[1];
    EncoderUpdates[1] = millis();
#ifdef ENCODER_DEBUG
    vLog("Encoder 1" + String(EncoderCounts[1]));
#endif
}
void EncoderISR2() { 
    EncoderCounts[2] += EncoderDirections[2];
    EncoderUpdates[2] = millis();
#ifdef ENCODER_DEBUG
    vLog("Encoder 2" + String(EncoderCounts[2]));
#endif
}
void EncoderISR3() {
    EncoderCounts[3] += EncoderDirections[3];
    EncoderUpdates[3] = millis();
#ifdef ENCODER_DEBUG
    vLog("Encoder 3" + String(EncoderCounts[3]));
#endif
}

const ISR EncoderISRs[] = {EncoderISR0, EncoderISR1, EncoderISR2, EncoderISR3};

#endif
