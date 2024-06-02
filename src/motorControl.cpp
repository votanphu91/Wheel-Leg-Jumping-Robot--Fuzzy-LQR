# include "motorControl.h"


void Leftmotor(int dir, uint8_t pwmValue){
    if(dir == 1){
        ledcWrite(channel_PWM1, pwmValue);
        ledcWrite(channel_PWM2, 0);
    }
    else if(dir == -1){
        ledcWrite(channel_PWM1,0);
        ledcWrite(channel_PWM2,pwmValue);
    }
}
void Rightmotor(int dir, uint8_t pwmValue){
    if(dir == 1){
        ledcWrite(channel_PWM3, pwmValue);
        ledcWrite(channel_PWM4, 0);
    }else if(dir == -1){
        ledcWrite(channel_PWM3,0);
        ledcWrite(channel_PWM4,pwmValue);
    }
}
void StopMotor(){
    ledcWrite(channel_PWM1,0);
    ledcWrite(channel_PWM2,0);
    ledcWrite(channel_PWM3,0);
    ledcWrite(channel_PWM4,0);
}
void motorControl(long leftPWM, long rightPWM, float angle_psi, bool stopstate){
    if(stopstate){
        StopMotor();
    }else{
        if(abs(angle_psi) > 30){
            StopMotor();
        }
        else{
            if(leftvolt > 0){
                Leftmotor(1, abs(leftPWM));
            }else if(leftvolt < 0){
                Leftmotor(-1, abs(leftPWM));
            }else{
                ledcWrite(channel_PWM1,0);
                ledcWrite(channel_PWM2,0);
            }

            if(righvolt > 0){
                Rightmotor(1, abs(rightPWM));
            }else if(righvolt < 0){
                Rightmotor(-1, abs(rightPWM));
            }else{
                ledcWrite(channel_PWM3,0);
                ledcWrite(channel_PWM4,0);
            }
        }
    }
}