#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "sensor_feedback.h"
#include "Adafruit_BNO055.h"

#include <stdint.h>

void stop();

void drive_straight(float set_angle, uint8_t speed, Adafruit_BNO055& imu);
void drive_straight_slow_pd(float set_angle, uint8_t speed, Adafruit_BNO055& imu);
void drive_straight_backwards(float set_angle, uint8_t speed, Adafruit_BNO055& imu);
void blind_drive(uint8_t speed);
void blind_reverse(uint8_t speed);
void turn(float turn_angle, uint8_t dir, uint8_t turn_speed, Adafruit_BNO055& imu);

void setupArdumoto();
void driveArdumoto(uint8_t motor, uint8_t dir, uint8_t spd);

#endif
