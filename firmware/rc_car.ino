#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>

Servo steer_servo;  
Servo throttle_servo; 

ros::NodeHandle nh;

int THROTTLE_CMD;
int STEER_CMD;

void velCallback(const std_msgs::Int32& vel)
{
   THROTTLE_CMD = vel.data;
   throttle_servo.write(THROTTLE_CMD);
   delay(15);
}

void steerCallback(const std_msgs::Int32& steer)
{
   STEER_CMD = steer.data;
   steer_servo.write(STEER_CMD);
   delay(15);
}

ros::Subscriber<std_msgs::Int32> sub_speed("/rc_car/speedPWM" , velCallback);
ros::Subscriber<std_msgs::Int32> sub_steer("/rc_car/steerPWM" , steerCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub_speed);
  nh.subscribe(sub_steer);
  steer_servo.attach(9); 
  steer_servo.write(90); 
  throttle_servo.attach(10);
  throttle_servo.write(115);
  nh.loginfo("Startup complete");
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
