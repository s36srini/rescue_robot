#include <stdint.h>
#include <Arduino.h>

#include "sensor_feedback.h"
#include "Adafruit_BNO055.h"

#define CW 1
#define CCW 0

// since our motor orientation is reversed
#define FORWARD 0
#define REVERSE 1

//delays
#define DESCEND_TIME 3500

// A is right, B is left
#define MOTOR_A 0
#define MOTOR_B 1

// Pin Assignments 
#define DIRA 2 // Direction control for motor A
#define PWMA 3  // PWM control (speed) for motor A
#define DIRB 4 // Direction control for motor B
#define PWMB 11 // PWM control (speed) for motor B

const float turn_tolerance = 6;

int d_pwm = 0;
float prev_t = 0;
float curr_t = 0;
float dt = 0;

const float Kp = 4.20, Ki = 0, Kd = 0.96; 

float accum_integral_error = 0; // Used for calculating integral error
float prev_error = 0; // Used for calculating derivative error

/** Predefined library functions **/

// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(uint8_t motor, uint8_t dir, uint8_t spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

// stopArdumoto makes a motor stop
void stopArdumoto(uint8_t motor)
{
   driveArdumoto(motor, 0, 0);
}

void setupArdumoto() {
     // All pins should be setup as outputs:     pinMode(PWMA, OUTPUT);
     pinMode(PWMB, OUTPUT);
     pinMode(DIRA, OUTPUT);
     pinMode(DIRB, OUTPUT);

     // Initialize all pins as low:
     digitalWrite(PWMA, LOW);
     digitalWrite(PWMB, LOW);
     digitalWrite(DIRA, LOW);
     digitalWrite(DIRB, LOW);
}
/*** End Predefined library functions ***/

void stop() {
    stopArdumoto(MOTOR_A);
    stopArdumoto(MOTOR_B);
}

// going straight at a target angle
void drive_straight(float target_angle, uint8_t speed, Adafruit_BNO055& imu) { // Ensure speed isn't above 250

    float raw_diff = get_yaw_reading(imu) - target_angle;
    float error = round(raw_diff + 180)%360 - 180;

    float total_error = Kp*error + Ki*(accum_integral_error + error) + Kd*(error - prev_error);
    
    accum_integral_error += error;
    prev_error = error;

    
    driveArdumoto(MOTOR_A, FORWARD, speed+total_error - 3);
    driveArdumoto(MOTOR_B, FORWARD, speed-total_error);
    
}

void drive_straight_slow_pd(float target_angle, uint8_t speed, Adafruit_BNO055& imu) { // Ensure speed isn't above 250

    float raw_diff = get_yaw_reading(imu) - target_angle;
    float error = round(raw_diff + 180)%360 - 180;

    float total_error = Kp*error + Ki*(accum_integral_error + error) + Kd*(error - prev_error);
    
    accum_integral_error += error;
    prev_error = error;

    
    driveArdumoto(MOTOR_A, FORWARD, speed+total_error - 3);
    driveArdumoto(MOTOR_B, FORWARD, speed-total_error);
    
}

void drive_straight_backwards(float target_angle, uint8_t speed, Adafruit_BNO055& imu) { // Ensure speed isn't above 250

    float raw_diff = get_yaw_reading(imu) - target_angle;
    float error = round(raw_diff + 180)%360 - 180;

    float total_error = Kp*error + Ki*(accum_integral_error + error) + Kd*(error - prev_error);
    
    accum_integral_error += error;
    prev_error = error;

    
    driveArdumoto(MOTOR_A, REVERSE, speed-total_error - 3);
    driveArdumoto(MOTOR_B, REVERSE, speed+total_error);
    
}

void blind_drive(uint8_t speed) {
      driveArdumoto(MOTOR_A, FORWARD, speed);
      driveArdumoto(MOTOR_B, FORWARD, speed);
}

void blind_reverse(uint8_t speed) {
      driveArdumoto(MOTOR_A, REVERSE, speed);
      driveArdumoto(MOTOR_B, REVERSE, speed);
}

void turn(float target_angle, uint8_t dir, uint8_t turn_speed, Adafruit_BNO055& imu){
  float curr_angle = get_yaw_reading(imu);
  float i = 0;
  uint8_t raw_speed;

  float diff = round(curr_angle - target_angle + 180)%360 - 180;
  
  while(abs(diff) > turn_tolerance) {      
    raw_speed = int(turn_speed*exp(i*-0.5) + 35);
    driveArdumoto(MOTOR_A, dir, raw_speed - 3);
    driveArdumoto(MOTOR_B, !dir,raw_speed);
    curr_angle = get_yaw_reading(imu);
    diff = round(curr_angle - target_angle + 180)%360 - 180;
    ++i;  
  }
  stop();
}
