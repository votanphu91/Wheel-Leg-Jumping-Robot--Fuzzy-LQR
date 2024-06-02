#ifndef ENCODER_H
#define ENCOEDR_H
// Định nghĩa các chân cho encoder
#include <Arduino.h>
#define ENCODER_1A 27   // 27
#define ENCODER_1B 14   // 14

#define ENCODER_2A 12 // Pin for Encoder A 12
#define ENCODER_2B 13 // Pin for Encoder  13

// Global variables
extern volatile long leftencoder;
extern volatile long rightencoder;
// Funtion prototypes
void encoder1_isr();
void encoder2_isr();

#endif