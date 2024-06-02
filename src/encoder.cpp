#include "encoder.h"

volatile long leftencoder = 0;      // position update by encoder
volatile long rightencoder = 0;      // position update by encoder
void encoder1_isr(){;
    int xung_1B = digitalRead(ENCODER_1B);
    if(xung_1B > 0){
        leftencoder++;
    }
    else{
        leftencoder--;
    }
}

void encoder2_isr(){;
    int xung_2B = digitalRead(ENCODER_2B);
    if(xung_2B >0){
        rightencoder++;
    }
    else{
        rightencoder--;
    }
}