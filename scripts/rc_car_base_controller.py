#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32 
from std_msgs.msg import Int32 
from ackermann_msgs.msg import AckermannDriveStamped
from math import pow, atan2, sqrt, sin, cos, radians

speedometer = 0.0

def convert_steer_angle_to_servo(theta):
    '''
    Maps steer angle setpoint to rc car steer servo position
    No angle sensor feedback
    Reads servo Min, Max and Center from params.
    Input range from -60 to 60 degrees.
    '''
    steer_setpoint = theta * 100
    steerServoPWM = valmap(steer_setpoint, -100, 100, STEERING_MIN, STEERING_MAX)

    return steerServoPWM


def convert_speed_to_servo(speed):
    '''
    Maps speed to rc_car esc.  The 'max speed' is set to 1.  Think about
    it like percent instead of meters per second.  The THROTTLE_MIN  will
    be top speed limit.  speed will be a percent of THROTTLE_MIN.

    If you want to go faster lower the Throttle_min
    '''
    speed_percent_setpoint = speed * 100
    if(speed_percent_setpoint > 0):
        speedServoPWM = valmap(speed_percent_setpoint, 0.0, 100.0, THROTTLE_ZERO, THROTTLE_MIN) 
    elif(speed_percent_setpoint < 0):
        speedServoPWM = valmap(speed_percent_setpoint, 0.0, -100.0, THROTTLE_ZERO, THROTTLE_MAX)
    else:
        speedServoPWM = THROTTLE_ZERO

    return speedServoPWM


# function to map range of input to range of servo motor
# Input -60 to 60 degrees
# Output read from param server
def valmap(value, istart, istop, ostart, ostop):

    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def msg_callback(data):
  
    global THROTTLE_MAX
    global THROTTLE_MIN
    global THROTTLE_ZERO
    global STERRING_MAX
    global STEERING_MIN
    global STEERING_ZERO

    theta = data.drive.steering_angle
    steerPWM_cmd = convert_steer_angle_to_servo(theta)

    speed = data.drive.speed
    speedPWM_cmd = convert_speed_to_servo(speed)

    pub_steer.publish(int(round(steerPWM_cmd)))
    pub_throttle.publish(int(round(speedPWM_cmd)))

def speed_callback(data):
    global speedometer
    speedometer = data.data


if __name__ == '__main__':
    try:

        rospy.init_node('rc_car_base_controller')

        THROTTLE_MAX = float(rospy.get_param('~throttle_max', '120.0'))
        THROTTLE_MIN = float(rospy.get_param('~throttle_min', '95.0'))
        THROTTLE_ZERO = float(rospy.get_param('~throttle_zero', '117.0'))

        STEERING_MAX = float(rospy.get_param('~steering_max', '114.0'))
        STEERING_MIN = float(rospy.get_param('~steering_min', '96.0'))
        STEERING_ZERO = float(rospy.get_param('~steering_zero', '100.0'))

        rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, msg_callback, queue_size=10)
        rospy.Subscriber('/rc_car/speedometer', Float32, speed_callback, queue_size=10)

        pub_speed = rospy.Publisher('/rc_car/cur_speed', Float32, queue_size=10)
        pub_speed_filtered = rospy.Publisher('/rc_car/speed_filtered', Float32, queue_size=10)

        pub_steer = rospy.Publisher('/rc_car/steerPWM', Int32, queue_size=10) 
        pub_throttle = rospy.Publisher('/rc_car/speedPWM', Int32, queue_size=10)

        rospy.loginfo("Node 'rc_car_base_controller' started. \nListening to 'Steer_max',  param value:  %s " % STEERING_MAX ) 


        rospy.spin()

    except rospy.ROSInterruptException:
        pass
