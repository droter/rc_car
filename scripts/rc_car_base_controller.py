#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32 
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped 
from ackermann_msgs.msg import AckermannDriveStamped
from math import pow, atan2, sqrt, sin, cos, radians
import numpy as np


speedometer = 0.0


def scale_angle_to_servo(angle):
    '''
    Maps steer angle setpoint to rc car steer servo position
    No angle sensor feedback
    Reads servo Min, Max and Center from params.
    Input range from -60 to 60 degrees.
    '''
    angle_setpoint = angle * 100
    steerServoPWM = valmap(angle_setpoint, -100, 100, STEERING_MIN, STEERING_MAX)

    return steerServoPWM


def scale_speed_to_servo(speed):
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


# function to scale range of input to range of servo motor min/max
def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def msg_callback(data):
    global THROTTLE_MAX
    global THROTTLE_MIN
    global THROTTLE_ZERO
    global STERRING_MAX
    global STEERING_MIN
    global STEERING_ZERO

    angle_cmd= data.drive.steering_angle
    steerPWM_cmd = scale_angle_to_servo(angle_cmd)

    speed_cmd = data.drive.speed
    speedPWM_cmd = scale_speed_to_servo(speed_cmd)

    pub_steer.publish(int(round(steerPWM_cmd)))
    pub_throttle.publish(int(round(speedPWM_cmd)))


# ExpMoving Average to smooth out instant speed
instant_speed_history = []
exp_speed_history = []
current_exp_speed = 0.0

def ExpMovingAverage(values, window):
    weights = np.exp(np.linspace(-1., 0., window))
    weights /= weights.sum()
    a = np.convolve(values, weights, mode='full')[:len(values)]
    a[:window] = a[window]
    return a


def speed_callback(vel_msg):
    global speedometer
    vel_x = vel_msg.twist.linear.x
    vel_y = vel_msg.twist.linear.y

    speedometer = sqrt(vel_x**2 + vel_y**2)

    # Filter speed estimate
    global exp_speed_history
    global current_exp_speed
    instant_speed_history.append(speedometer)

    if (len(instant_speed_history) > 20):
        exp_speed_history = ExpMovingAverage(instant_speed_history, 5)
        current_exp_speed = float(exp_speed_history[-1])
        instant_speed_history.pop(0)
        exp_speed_history = np.delete(exp_speed_history, 1)

    pub_speed.publish(float(speedometer))
    pub_exp_speed.publish(current_exp_speed)


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
        rospy.Subscriber('/vel', TwistStamped, speed_callback, queue_size=10)

        pub_speed = rospy.Publisher('/rc_car/instant_speed', Float32, queue_size=10)
        pub_exp_speed = rospy.Publisher('/rc_car/exp_instant_speed', Float32, queue_size=10)

        pub_steer = rospy.Publisher('/rc_car/steerPWM', Int32, queue_size=10) 
        pub_throttle = rospy.Publisher('/rc_car/speedPWM', Int32, queue_size=10)

        # rospy.loginfo("Node started. \nListening to 'Steer_max',  param value:  %s " % STEERING_MAX ) 
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
