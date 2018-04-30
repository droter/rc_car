#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Twist.h>

// create servo object to control servo
Servo steer_servo;  
Servo throttle_servo; 

float THROTTLE_SETPOINT;
float STEER_SETPOINT;
float THROTTLE_CMD;
float STEER_CMD;

// Calibration values for vehicle
float THROTTLE_ZERO = 118;
float THROTTLE_MAX = 130;
float THROTTLE_MIN = 70;
float STEER_ZERO = 100;
float STEER_MAX = 146;
float STEER_MIN = 54;

ros::NodeHandle nh;


void velCallback( const geometry_msgs::Twist& motion)
{
/*
 *  Read linear.x from /cmd_vel topic and write to output
 *  RC Car with ESC motor controller
 *  
*/
   THROTTLE_SETPOINT = (motion.linear.x * 100);

    if(THROTTLE_SETPOINT > 0)
    { 
      THROTTLE_CMD = map(THROTTLE_SETPOINT, 0, 100, THROTTLE_ZERO, THROTTLE_MAX );
    }
     else if(THROTTLE_SETPOINT < 0)
    {
      THROTTLE_CMD = map(THROTTLE_SETPOINT, 0, -100, THROTTLE_ZERO, THROTTLE_MIN );
    }
    else
    {
      //return throttle to zero if there's no command
      THROTTLE_CMD = THROTTLE_ZERO;
    }

/*
*  Read angular.z value from /cmd_vel topic and write to output
*  
*/
   STEER_SETPOINT = (motion.angular.z * 100);       //steering
   
    if(STEER_SETPOINT > 0)
    {
        //steer left 
        STEER_CMD = map(STEER_SETPOINT, 0, 100, STEER_ZERO, STEER_MIN);
    }
    else if(STEER_SETPOINT < 0)
    {
        //steer right
        STEER_CMD = map(STEER_SETPOINT, 0, -100, STEER_ZERO, STEER_MAX);
    }
    else
    {
        //return steering wheel to middle if there's no command
        STEER_CMD = STEER_ZERO;
    }

    // Print to ROS Log
    //char infoVal[10];
    //dtostrf(THROTTLE_CMD, 4, 3, infoVal);
    //nh.loginfo(infoVal);

/*
*  Write values to servos
*  
*/
    throttle_servo.write(THROTTLE_CMD);
    steer_servo.write(STEER_CMD);
    delay(15);

    
  }

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel" , velCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  steer_servo.attach(9); 
  steer_servo.write(STEER_ZERO); 
  throttle_servo.attach(10);
  throttle_servo.write(THROTTLE_ZERO);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
