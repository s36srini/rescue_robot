#include <stdint.h>
#include <Servo.h>

#include "servo_control.h"

void pull_back_servos(Servo& servo_left, Servo& servo_right) {
  servo_left.write(160);
  servo_right.write(10);
}

void drop_servos(Servo& servo_left, Servo& servo_right) {
  float left_angle = servo_left.read();
  float right_angle = servo_right.read();
  float turn_angle = 50;

  servo_left.write(left_angle - turn_angle - 12);
  servo_right.write(right_angle + turn_angle - 10);
}

void setup_servos(Servo& servo_left, Servo& servo_right, int left_servo_pin, int right_servo_pin){
  servo_left.attach(left_servo_pin);
  servo_right.attach(right_servo_pin);

  pull_back_servos(servo_left, servo_right);
}
