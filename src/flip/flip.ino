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

void flip(){
  servo.write(close_angle);
  delay(2000);

  digitalWrite(act_ext_pin, LOW);
  digitalWrite(act_ret_pin, HIGH);
  delay(2000);
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, HIGH);
  delay(1000);

  //CHECK CURRENT

  digitalWrite(act_ext_pin, LOW);
  digitalWrite(act_ret_pin, HIGH);
  delay(6000);
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, HIGH);
  delay(1000);

  servo.write(open_angle);
  delay(2000);

  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, LOW);
  delay(5000);
  digitalWrite(act_ext_pin, HIGH);
  digitalWrite(act_ret_pin, HIGH);
  delay(1000);
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

ros::Publisher pub("flip_done", &flipped_msg);

void flip_cb( const std_msgs::Bool& cmd_msg){
  flip();
  
  flipped_msg.data = true;
  pub.publish(&flipped_msg);
  delay(1000);
}


ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> sub("flipper_start", flip_cb);


void setup(){
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
  pinMode(act_ext_pin, OUTPUT);
  pinMode(act_ret_pin, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(1);
  //flip();
  //actuator_test();
}
