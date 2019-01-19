#include <stdint.h>
#include <math.h>
#include<Filters.h>

#include "sensor_feedback.h"
#include "Adafruit_VL53L0X.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define OUT_OF_RANGE (2<<16) - 1 // Defines out of range value (MAX_INT) for ToF sensor reading

#define POLE_THRESHOLD 180 //180  //blank wall is about 200, pole is about 170
#define HIGH_PASS_THRESHOLD 9 //old 90
 
uint16_t prev_ultrasonic_reading = 0;

//FIlter parameters
const float filter_frequency = 1;
FilterOnePole low_pass_filter(LOWPASS, filter_frequency);

// Moving average
const int cache_size = 5;
//uint16_t ultrasonic_cache[cache_size] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t ultrasonic_cache[cache_size] = {0,0,0,0,0};
uint16_t prev_ultrasonic_sum = 0;

/** IMU functions **/

float get_yaw_reading(Adafruit_BNO055& imu){
  sensors_event_t event;
  imu.getEvent(&event);

  return event.orientation.x;
 }

float get_pitch_reading(Adafruit_BNO055& imu) {
  
  sensors_event_t event;
  imu.getEvent(&event);
  
  float val = event.orientation.z;
  return val;
}

/** ToF functions **/
uint16_t get_tof_dist_reading(Adafruit_VL53L0X& tof) {
    VL53L0X_RangingMeasurementData_t measure;
    
    tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    uint16_t reading;
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      reading = measure.RangeMilliMeter;
    } else {
      reading = OUT_OF_RANGE;
    }

    return reading;
}

uint16_t get_tof_dist_reading_avg(Adafruit_VL53L0X& tof) {
  uint16_t sum = 0;
  int num_faults = 0;

  for(int i = 0; i < 5; ++i) {
    uint16_t reading = get_tof_dist_reading(tof);
    if(reading != OUT_OF_RANGE)  {
      sum += reading;
    } else {
      ++num_faults;
    }
  }

  if(num_faults == 5) {
    return OUT_OF_RANGE;
  }

  return sum / (5 - num_faults);
}


/** Ultrasonic Functions **/
void setup_ultrasonic(const int trig_pin, const int echo_pin){
  pinMode(trig_pin, OUTPUT);
  digitalWrite(echo_pin, LOW);
}

uint16_t get_ultrasonic_reading(const int trig_pin, const int echo_pin){

  const unsigned int MAX_DIST = 23200;
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(echo_pin) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(echo_pin) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

  // Print out results
  
//  if ( pulse_width > MAX_DIST ) {
//    Serial.println("Out of range");
//  } else {
//    Serial.print(cm);
//    Serial.print(" cm \n");
//  }

  // Wait at least 60ms before next measurement
  delay(60);

  return cm;
}

uint16_t get_ultrasonic_reading_moving_avg(const int trig_pin, const int echo_pin) {
  uint16_t new_reading = get_ultrasonic_reading(trig_pin, echo_pin);
  uint16_t oldest_reading = ultrasonic_cache[cache_size - 1];

  for(int i = cache_size - 1; i > 0; --i) {
    ultrasonic_cache[i] = ultrasonic_cache[i-1];
  }

  ultrasonic_cache[0] = new_reading;
  prev_ultrasonic_sum = (prev_ultrasonic_sum - oldest_reading + new_reading);

  return prev_ultrasonic_sum / cache_size;

  
}

bool is_pole_detected_old(const int trig_pin, const int echo_pin) {
  uint16_t reading = get_ultrasonic_reading_max(trig_pin, echo_pin);
  
  if(reading < POLE_THRESHOLD && reading > HIGH_PASS_THRESHOLD) {
      return true;
  }
  return false;
}

bool is_pole_detected(const int trig_pin, const int echo_pin) { // Checks to see whether a pole is detected
  uint16_t average_reading = get_ultrasonic_reading_moving_avg(trig_pin, echo_pin);

  uint16_t variance = 0;
  for(int i = 0; i < cache_size; ++i) {
    variance += pow(average_reading - ultrasonic_cache[i], 2);
  }

  if(sqrt(variance) < 15 && average_reading < POLE_THRESHOLD) {
    for(int i = 0; i < cache_size; i++) {
      ultrasonic_cache[i] = 0;
    }
    prev_ultrasonic_sum = 0;
    return true;
  }

  return false;
}

bool is_pole_detected_filtered(const int trig_pin, const int echo_pin){
  uint16_t reading = get_ultrasonic_reading(trig_pin, echo_pin);
  //low_pass_filter.input(reading);
  Serial.print(reading);
  Serial.print(",");
  //reading = low_pass_filter.output();
  Serial.println(reading);
  
  if(reading < POLE_THRESHOLD && reading > HIGH_PASS_THRESHOLD) {
    if(abs(reading - prev_ultrasonic_reading) < 10) {
        prev_ultrasonic_reading = 0;
        return true;
    } else if(abs(reading - prev_ultrasonic_reading) > 100){
        return false;
    }
     else {
      prev_ultrasonic_reading = reading;
    }
  }
  
  return false;
}

bool is_pole_detected_sorted(const int trig_pin, const int echo_pin){
  uint16_t reading = get_ultrasonic_reading_midpoint(trig_pin, echo_pin);
  
  if(reading < POLE_THRESHOLD && reading > HIGH_PASS_THRESHOLD) {
    if(abs(reading - prev_ultrasonic_reading) < 20) {
        prev_ultrasonic_reading =  0;
        return true;
    } 
  }
  prev_ultrasonic_reading = reading;
  return false;
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}


uint16_t get_ultrasonic_reading_midpoint(const int trig_pin, const int echo_pin) {
  uint16_t new_reading = get_ultrasonic_reading(trig_pin, echo_pin);
  uint16_t oldest_reading = ultrasonic_cache[cache_size - 1];

  for(int i = cache_size - 1; i > 0; --i) {
    ultrasonic_cache[i] = ultrasonic_cache[i-1];
  }

  ultrasonic_cache[0] = new_reading;
  qsort(ultrasonic_cache, cache_size, sizeof(ultrasonic_cache[0]),sort_desc);

  return ultrasonic_cache[cache_size/2];
}

//meant to minimize effect of random drops in ultrasonic reading
uint16_t get_ultrasonic_reading_max(const int trig_pin, const int echo_pin){
  uint16_t max = get_ultrasonic_reading(trig_pin, echo_pin);

  for(int i = 0; i < 2; ++i) {
    uint16_t reading = get_ultrasonic_reading(trig_pin, echo_pin);
    if(reading > max)  {
      max = reading;
    }
  }

  return max;
}
