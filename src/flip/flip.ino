/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

int servo_pin1 = 9;
int servo_pin2 = 10;
int current_pin1 = A0;
int act_ext_pin = 8;
int act_ret_pin = 7;

int buttonPin1 = 5;
int buttonPin2 = 4;
int buttonPin3 = 3;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;

const int RS = 10;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read
const int CURRENT_THRES = 0.1;
const int CURRENT_COUNTER = 0;

// Global Variables
float sensorValue;   // Variable to store value from analog read
float current;
bool flipping = false;


int open_angle = 0;
int close_angle = 140;

std_msgs::Bool flipped_msg;
std_msgs::Float32 current_msg;
Servo servo1;
Servo servo2;
ros::NodeHandle nh;
ros::Publisher flip_pub("/boat_controller/flip/arduino/done", &flipped_msg);
ros::Publisher curr_pub("/boat_controller/flip/current/", &current_msg);


void wait_blocked(int duration, int frame=1000){
  int rem_dur = duration;
  while (rem_dur > frame) {
    nh.spinOnce();
    delay(frame);
    rem_dur = rem_dur - frame;
  }
  
  nh.spinOnce();
  delay(rem_dur);
}

void wait(unsigned long duration){
  unsigned long start = millis();
  while (millis() - start < duration) {
    nh.spinOnce();
  }
}

void ret_actuator(unsigned long duration){
  digitalWrite(act_ext_pin, LOW);
  digitalWrite(act_ret_pin, HIGH);
  wait(duration);
}

void ext_actuator(unsigned long duration){
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, LOW);
  wait(duration);
}

void brake_actuator(unsigned long duration){
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, HIGH);
  wait(duration);
}

void open_servo(int start_angle, int end_angle){
  for (int pos = start_angle; pos <= end_angle; pos += 1) {
    servo1.write(pos);
    servo2.write(180-pos);
    nh.spinOnce();
    delay(5);
  }
  delay(1000);
}

void close_servo(int start_angle, int end_angle){
  for (int pos = start_angle; pos >= end_angle; pos -= 1) {
    servo1.write(pos);
    servo2.write(180-pos);
    nh.spinOnce();
    delay(5);
  }
}

float check_current() {
  //CHECK CURRENT
  sensorValue = analogRead(current_pin1);

  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  float curr = sensorValue / (1 * RS);
  return curr;
}

void flip(){
//  open_servo(open_angle, close_angle);
  
  servo1.write(close_angle);
  servo2.write(180-close_angle);
  delay(1000);

  ret_actuator(4000);
  brake_actuator(100);

  ret_actuator(9000);
  brake_actuator(100);

//  close_servo(close_angle, open_angle);
  servo1.write(180-close_angle);
  servo2.write(close_angle);

  ext_actuator(9000);
  brake_actuator(100);

  flipping = false;
  
//  }
//  else {
//    reset(1);
//  }
}

void reset(int x){
  close_servo(close_angle, open_angle);
  if (x == 1){  
    ext_actuator(8000); 
  }
  else {
    ret_actuator(8000);
  }
  brake_actuator(100);
  flipping = false;
}

void flip_cb( const std_msgs::UInt16& cmd_msg){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
  if (cmd_msg.data == 0){
    flip();
    flipped_msg.data = true;
  }
  else {
    int x = cmd_msg.data;
    reset(x);
    flipped_msg.data = false;
  }
  flip_pub.publish(&flipped_msg);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}


ros::Subscriber<std_msgs::UInt16> sub("/boat_controller/flip/arduino/start", flip_cb);


void setup(){
  nh.initNode();
  nh.advertise(flip_pub);
  nh.advertise(curr_pub);
  nh.subscribe(sub);

  pinMode(LED_BUILTIN, OUTPUT);
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10);
  pinMode(act_ext_pin, OUTPUT);
  pinMode(act_ret_pin, OUTPUT);
//  Serial.begin(9600);
}

void loop(){
  nh.spinOnce();

  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);

//  Serial.println(flipping);
//  Serial.println(buttonState1);
//  Serial.println(buttonState2);
//  Serial.println(buttonState3);
//  Serial.println(" ");

  if (flipping == false) {
    if (buttonState1 == HIGH) {
//      Serial.println("FLIP1!");
      flipping = true;
      flip();
    } else if (buttonState2 == HIGH) {
//      Serial.println("FLIP2!");
      flipping = true;
      reset(1);
    } else if (buttonState3 == HIGH) {
//      Serial.println("FLIP3!");
      flipping = true;
      reset(2);
    }
    buttonState1 = 0;
    buttonState2 = 0;
    buttonState3 = 0;
  }
  
  delay(1);
  
}
