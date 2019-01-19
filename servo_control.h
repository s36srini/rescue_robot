#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Servo.h>


void pull_back_servos(Servo& servo_left, Servo& servo_right);
void drop_servos(Servo& servo_left, Servo& servo_right); 
void setup_servos(Servo& servo_left, Servo& servo_right, int left_servo_pin, int right_servo_pin);


#endif

