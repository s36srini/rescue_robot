#ifndef SENSOR_FEEDBACK_H
#define SENSOR_FEEDBACK_H

#include <stdint.h>

#include "Adafruit_VL53L0X.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

float get_yaw_reading(Adafruit_BNO055& imu);
float get_pitch_reading(Adafruit_BNO055& imu);

uint16_t get_tof_dist_reading(Adafruit_VL53L0X& tof);
uint16_t get_tof_dist_reading_avg(Adafruit_VL53L0X& tof);

void setup_ultrasonic(const int tirg_pin, const int echo_pin);
uint16_t get_ultrasonic_reading(const int tirg_pin, const int echo_pin);
uint16_t get_ultrasonic_reading_max(const int tirg_pin, const int echo_pin);
uint16_t get_ultrasonic_reading_moving_avg(const int trig_pin, const int echo_pin);
uint16_t get_ultrasonic_reading_midpoint(const int trig_pin, const int echo_pin);

bool is_pole_detected_old(const int trig_pin, const int echo_pin);
bool is_pole_detected(const int trig_pin, const int echo_pin);
bool is_pole_detected_filtered(const int trig_pin, const int echo_pin);
bool is_pole_detected_sorted(const int trig_pin, const int echo_pin);

#endif
