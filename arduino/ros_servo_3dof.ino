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

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
//#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>


ros::NodeHandle  nh;

Servo Bservo,Mservo,Uservo;

int servo_b = 9;
int servo_m = 10;
int servo_u = 11;


void servo_cb( const std_msgs::Int16MultiArray& cmd_msg){

  int b_angle = cmd_msg.data[0], m_angle = cmd_msg.data[1], u_angle = cmd_msg.data[2];
  
  Bservo.write(b_angle);
  Mservo.write(m_angle);
  Uservo.write(u_angle);
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  Bservo.attach(servo_b);
  Mservo.attach(servo_m);
  Uservo.attach(servo_u);
}

void loop(){
  
  nh.spinOnce();
  delay(15);
}
