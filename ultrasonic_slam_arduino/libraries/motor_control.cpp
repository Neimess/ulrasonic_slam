#include "motor_control.h"

void setupMotors() {
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void move(int speed, float time, char* direction){
    time = time * 1000;
    int speed_left = speed;
    int speed_right = speed-10;
    analogWrite(enA, speed_right);
    analogWrite(enB, speed);
    switch (*direction) {
        case 'F':
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            break;
        case 'B':
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            break;
    } 
    delay(time);
}

void rotate(float angle) {
    analogWrite(enA, 255);
    analogWrite(enB, 245);
    float proportion = abs(angle) / 90.0;
    if (angle <= 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    delay(delayTime);
}

void stopMotors(int time){
    time = time * 1000;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(time);
}

