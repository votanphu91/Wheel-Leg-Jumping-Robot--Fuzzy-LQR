#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>
// Định nghĩa các chân cho L298
#define IN1 32          
#define IN2 33
#define IN3 25
#define IN4 26

extern int channel_PWM1;        // su dung 2 kenh de kiem soat huong va toc do cua dong co , // PWM voi 8 kenh dau voi tan so 80MHz (high-speed channel), và 8 kênh cuối clock 1MHz (low-speed channel).
extern int channel_PWM2;       // co 16 kenh ledc co san, ta co the lua chon bat ki tu 0 - 15
extern int channel_PWM3;
extern int channel_PWM4;
// Dien ap cap banh trai va banh phai 

extern float leftvolt;
extern float righvolt;

// Funtion prototypes
void Rightmotor(int dir, uint8_t pwmValue);
void Leftmotor(int dir, uint8_t pwmValue);
void StopMotor();
void motorControl(long leftPWM, long rightPWM, float angle_psi, bool stopstate);
#endif