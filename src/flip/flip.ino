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

int servo_pin = 9;
int act_ext_pin = 8;
int act_ret_pin = 7;

int open_angle = 10;
int close_angle = 170;

std_msgs::Bool flipped_msg;
Servo servo;
ros::NodeHandle nh;
ros::Publisher pub("/boat_controller/flip/arduino/done", &flipped_msg);

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
    servo.write(pos);
    nh.spinOnce();
    delay(1);
  }
}

void close_servo(int start_angle, int end_angle){
  for (int pos = start_angle; pos >= end_angle; pos -= 1) {
    servo.write(pos);
    nh.spinOnce();
    delay(1);
  }
}


void flip(){
  open_servo(open_angle, close_angle);
  wait(3000);

  ret_actuator(3000);
  brake_actuator(2000);

  //CHECK CURRENT

  ret_actuator(7000);
  brake_actuator(2000);

  close_servo(close_angle, open_angle);
  wait_blocked(2000);

  ext_actuator(5000);
  brake_actuator(2000);
}

void actuator_test(){
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, LOW);
  delay(2000);

  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, HIGH);
  delay(2000);

//  digitalWrite(act_ext_pin, LOW);
//  digitalWrite(act_ret_pin, HIGH);
//  delay(10000);
//
//  digitalWrite(act_ext_pin, HIGH);
//  digitalWrite(act_ret_pin, HIGH);
//  delay(1000);
}

void flip_cb( const std_msgs::Bool& cmd_msg){
  flip();
  
  flipped_msg.data = true;
  pub.publish(&flipped_msg);
  delay(1000);
}


ros::Subscriber<std_msgs::Bool> sub("/boat_controller/flip/arduino/start", flip_cb);


void setup(){
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
  pinMode(act_ext_pin, OUTPUT);
  pinMode(act_ret_pin, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(1);
  //actuator_test();

  

}
