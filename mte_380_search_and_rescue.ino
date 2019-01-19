#include <stdint.h>
#include <math.h>

#include <Servo.h>

#include "motor_control.h"
#include "servo_control.h"
#include "sensor_feedback.h"
#include "Adafruit_VL53L0X.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CW 1
#define CCW 0
#define TURN_SPEED 27

#define DECELERATION_DISTANCE 150 //slowing distance

#define ASCEND_ANGLE 15
#define FLATTEN_ANGLE 2
#define DESCEND_ANGLE -28
#define GROUND_ANGLE -25

#define DESCEND_TIME 2100 //try 1900, 2100

#define BLIND_DRIVE_TIME 5000 //in milliseconds
#define BLIND_DRIVE_TIME_2 2250

#define HOME_ANGLE 35

Adafruit_VL53L0X tof_sensor = Adafruit_VL53L0X(); // Setup ToF sensor
Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55); // Setup IMU sensor

Servo servo_l, servo_r;

const int initial_wall_distance_1 = 169; // Distance representing robot's desired distance away from the wall, turn happens subsequently when this is reached. Measured 100
const int initial_wall_distance_2 = 145; //used to be 200

const int turning_wall_distance_1 = 250; // Distance representing robot's desired distance away from the wall when about to turn onto ramp. Original 335, measured 166
const int turning_wall_distance_2 = 200;  //about 5 cm less?

const int turning_wall_distance_1_charged = 260;  //try 300, 275, 250
const int turning_wall_distance_2_charged = 145;  //try 200, 250

const int close_wall_distance = 110;

const int ultrasonic_wall_distance = 340;  // Distance representing robot's desired distance away from the wall after descending the ramp and turning to look for the pole
const int pole_distance = 150; //Distance representing impact with target

//robot speeds
const int floor_speed = 71;
const int ramp_up_speed = 160; //old 220
const int ramp_top_speed = 69;

//ultrasonic pins
const int left_sonic_trig = 39;
const int left_sonic_echo = 41;
const int right_sonic_trig = 43;
const int right_sonic_echo = 45;

//servo pins
const int left_servo_pin = A4;
const int right_servo_pin = A5;

//push button pin
const int push_button = 13;

//limit switch
const int l_switch = 23;

float yaw = 0;
float target_angle = 0;

bool is_edge_case = false;  //true if the pole is in the zone of the bot's travel after turning

void setup_button(int button_pin){
  pinMode(button_pin, INPUT);
}

int get_button_val(int button_pin){
  int val = digitalRead(button_pin);
  return val;
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial);

  setup_button(push_button);
  pinMode(52,OUTPUT); // LED pin

  setupArdumoto(); // set all pins as outputs
  
  setup_ultrasonic(left_sonic_trig, left_sonic_echo);
  setup_ultrasonic(right_sonic_trig, right_sonic_echo);
  
  Serial.println("Adafruit VL53L0X test");
  if (!tof_sensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    //while(1);
  }
  if (!imu_sensor.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  digitalWrite(52, HIGH);
  delay(1000);
  digitalWrite(52, LOW);

  setup_servos(servo_l, servo_r, left_servo_pin, right_servo_pin);
}

void update_target_angle(float angle, uint8_t dir) {
  if(dir == CW) {
    target_angle += angle;
    if (target_angle >= 360) target_angle -= 360;
  }
  else if(dir == CCW) {
    target_angle -= angle;
    if (target_angle < 0) target_angle += 360;
  }
}

void update_and_turn(float turn_angle, uint8_t dir, uint8_t turn_speed, Adafruit_BNO055& imu){
  update_target_angle(turn_angle, dir);
  turn(target_angle, dir, TURN_SPEED, imu_sensor);
}

void drive_straight_until_wall(float target_angle, Adafruit_BNO055& imu, int distance) {
  while(get_tof_dist_reading(tof_sensor) > distance) {
    drive_straight(target_angle, floor_speed, imu_sensor); 
  }
}

void drive_straight_slowing(float target_angle, Adafruit_BNO055& imu, Adafruit_VL53L0X& tof, uint16_t distance) {
  uint8_t speed = floor_speed;

  float i = 0;
  while(get_tof_dist_reading(tof_sensor) > distance) {
    speed = int(floor_speed*exp(i*-0.2) + 41);
    drive_straight(target_angle, speed, imu_sensor);
    ++i;
  }
}

void drive_straight_timed(float target_angle, Adafruit_BNO055& imu, int totalTime) {
  float startTime = millis();
  while(millis() - startTime < totalTime) {
    drive_straight(target_angle, floor_speed, imu_sensor);
  }
}

void first_state(uint8_t dir) {
    if(!is_edge_case) {
      drive_straight_until_wall(target_angle, imu_sensor, initial_wall_distance_1);
      stop();
      delay(500);
      update_and_turn(90, dir, TURN_SPEED, imu_sensor);
      stop();
      delay(500);
      
    }

    if(dir == CCW) {
      drive_straight_slowing(target_angle, imu_sensor, tof_sensor, turning_wall_distance_1_charged);
    }
    else if(dir == CW) {
      drive_straight_slowing(target_angle, imu_sensor, tof_sensor, turning_wall_distance_2_charged);
    }

    stop();
    delay(500);
    
    update_and_turn(90, dir, TURN_SPEED, imu_sensor);

    stop();
    delay(500);

    is_edge_case = false;
}

void second_state(bool isFirstTime) {

    unsigned long start_time = millis();
  Serial.println("Starting State 2");
  while(get_pitch_reading(imu_sensor) < ASCEND_ANGLE || millis() - start_time < 150) {
    drive_straight(target_angle, floor_speed, imu_sensor);
  }
  Serial.println("Stuck beforegettingonramp");

  drop_servos(servo_l, servo_r);

//  while(millis() - start_time < 3000){
//    blind_drive(ramp_up_speed/2);
//  }
//
//    Serial.println("Stuck after ascending 2");

  
  while(get_pitch_reading(imu_sensor) > FLATTEN_ANGLE || millis() - start_time < 200) {
    blind_drive(ramp_up_speed);
  }

    //Serial.println("Stuck after flattening");


  start_time = millis();
  while(get_pitch_reading(imu_sensor) > DESCEND_ANGLE || millis() - start_time < 200){
    drive_straight(target_angle, ramp_top_speed, imu_sensor);
  }

    //Serial.println("Stuck after goingdown");


  int descend_time = isFirstTime ? DESCEND_TIME : DESCEND_TIME - 1500;
  int reverse_speed = isFirstTime ? 225 : 255;

  start_time = millis();
  unsigned long target_time = millis() + descend_time;
//  while(millis() < target_time && get_pitch_reading(imu_sensor) < GROUND_ANGLE) {
  while(get_pitch_reading(imu_sensor) < GROUND_ANGLE || millis() - start_time < 200) {
    blind_reverse(reverse_speed);
  }

    //Serial.println("Stuck after blindrevers");


  pull_back_servos(servo_l, servo_r);

  delay(50);
  while(get_pitch_reading(imu_sensor) < -5 || millis() - start_time < 150){
    drive_straight(target_angle, 35, imu_sensor);
  }

    //Serial.println("Stuck after endof goingdown");


  stop();
  delay(1000);
  imu_sensor.begin();
  target_angle = 0;
  setupArdumoto();
  delay(500);
}


void third_state(uint8_t dir) {
  int sonic_trig;
  int sonic_echo;
  int after_ramp_distance;

  if(dir == CW) {
    sonic_trig = right_sonic_trig;
    sonic_echo = right_sonic_echo;
    after_ramp_distance = initial_wall_distance_1;
  }
  else if(dir == CCW){
    sonic_trig = left_sonic_trig;
    sonic_echo = left_sonic_echo;
    after_ramp_distance = initial_wall_distance_2;
  }

  while(get_tof_dist_reading(tof_sensor) > after_ramp_distance) {
    drive_straight(target_angle, floor_speed, imu_sensor);
    if(is_pole_detected_old(sonic_trig, sonic_echo)) {
      is_edge_case = true;
      break;
    }
  }
  stop();
  delay(500);

  if(is_edge_case){
    update_and_turn(90, dir, TURN_SPEED, imu_sensor);
    stop();
    delay(500);
    
    unsigned long startTime = millis();
    while(millis() - startTime < BLIND_DRIVE_TIME) {
      drive_straight(target_angle, floor_speed, imu_sensor);
      if(get_tof_dist_reading(tof_sensor) < pole_distance  || !digitalRead(l_switch)) {
        break;
      }
    }

    stop(); 
    delay(500);
  }
  else {
    update_and_turn(90, dir, TURN_SPEED, imu_sensor);  
    stop();
    delay(500); 
    drive_straight_timed(target_angle, imu_sensor, 1250);
  
    while (!is_pole_detected_old(sonic_trig, sonic_echo)) {
      drive_straight(target_angle, floor_speed, imu_sensor);
    }

    stop();
    delay(500);
    unsigned long startTime = millis();
    while(millis() - startTime < 300) {
      drive_straight_backwards(target_angle, floor_speed, imu_sensor);
    }
    stop();
    delay(500);
    update_and_turn(90, dir, TURN_SPEED, imu_sensor);
    stop();
    delay(500);

    int blind_drive_time = dir == CCW ? BLIND_DRIVE_TIME : 1500;
    startTime = millis();
    while(millis() - startTime < blind_drive_time) {
      drive_straight(target_angle, floor_speed, imu_sensor);
      if(get_tof_dist_reading(tof_sensor) < pole_distance || !digitalRead(l_switch)) {
        break;
      }
    }
    
//    uint16_t last_reading = 0;
//    uint16_t curr_reading = get_tof_dist_reading_avg(tof_sensor);
//    while(abs(last_reading - curr_reading) > 5) {
//      last_reading = curr_reading;
//      long curr_time = millis();
//      long delay_time = curr_time + 500;
//      while(curr_time < delay_time) {
//         drive_straight(target_angle, floor_speed, imu_sensor);
//         curr_time = millis();
//      }
//      curr_reading = get_tof_dist_reading(tof_sensor);
//    }

    stop();
  }

  delay(500);

  if(dir == CCW) {
    update_and_turn(180, dir, TURN_SPEED, imu_sensor);
    stop();
    delay(500);
  }
}

void go_home(){
  while(get_tof_dist_reading(tof_sensor) > initial_wall_distance_1) {
    drive_straight(target_angle, floor_speed, imu_sensor);
  }
  stop();
  update_and_turn(90, CW, TURN_SPEED, imu_sensor);
  update_and_turn(HOME_ANGLE, CW, TURN_SPEED, imu_sensor);

  unsigned long startTime = millis();
  while(millis() - startTime < BLIND_DRIVE_TIME_2) {
    drive_straight(target_angle, floor_speed, imu_sensor);
  } 
}


void loop() {
//  int prev = 0;
//  int curr = 0;
//  while(1){
//    is_pole_detected_filtered(left_sonic_trig, left_sonic_echo);
//  }

  while(!get_button_val(push_button)); // detecting push
  while(get_button_val(push_button));// detecting release


  first_state(CCW);
  second_state(true);
  third_state(CCW);
  first_state(CW);
  second_state(false);
//  third_state(CW);
  go_home();
  
  stop();
}
