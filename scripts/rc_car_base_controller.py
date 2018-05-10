#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32 
from std_msgs.msg import Int32 
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from math import pow, atan2, sqrt, sin, cos, radians
from sensor_msgs.msg import NavSatFix
import numpy as np

last_pos_lat = 0.0
last_pos_lon = 0.0
last_gps_time = 0.0
speed_estimates = []
exp_speed_est = []
current_est_speed = 0.0


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


def ExpMovingAverage(values, window):
    weights = np.exp(np.linspace(-1., 0., window))
    weights /= weights.sum()
    a = np.convolve(values, weights, mode='full')[:len(values)]
    a[:window] = a[window]
    return a


def gps_callback(gps):
    #callback every time the robot's gps filtered is received
    #calculate current speed from Odom.  No encoders

    global last_pos_lat
    global last_pos_lon
    global last_gps_time

    # approx radius of earth in km
    R = 6373.0

    current_time = rospy.get_time()

    cur_pos_lat = radians(gps.latitude)
    cur_pos_lon = radians(gps.longitude)

    delta_time = (current_time - last_gps_time)
    last_gps_time = current_time

    # get distance
    dlat = last_pos_lat - cur_pos_lat
    dlon = last_pos_lon - cur_pos_lon

    a = sin(dlat / 2)**2 + cos(cur_pos_lat) * cos(last_pos_lat) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    # distance in meters 
    distance = (R * c) * 1000

    current_speed = distance / delta_time

    # record last position
    last_pos_lat = cur_pos_lat
    last_pos_lon = cur_pos_lon

    pub_speed.publish(current_speed)
    rospy.loginfo("Current speed  %s", str(current_speed))

    # Filter speed estimate
    global exp_speed_est
    global current_est_speed
    speed_estimates.append(current_speed)
    rospy.loginfo(" estimate  %s", str(len(speed_estimates)))
    
    if (len(speed_estimates) > 20):
        exp_speed_est = ExpMovingAverage(speed_estimates, 15)
        current_est_speed = float(exp_speed_est[-1])
        speed_estimates.pop(0)
        exp_speed_est = np.delete(exp_speed_est, 1)

    pub_speed_filtered.publish(current_est_speed)
    
    rospy.loginfo("Exp speed estimate  %s", str(exp_speed_est))

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
        rospy.Subscriber('/gps/filtered', NavSatFix, gps_callback, queue_size=10)

        pub_speed = rospy.Publisher('/rc_car/cur_speed', Float32, queue_size=10)
        pub_speed_filtered = rospy.Publisher('/rc_car/speed_filtered', Float32, queue_size=10)

        pub_steer = rospy.Publisher('/rc_car/steerPWM', Int32, queue_size=10) 
        pub_throttle = rospy.Publisher('/rc_car/speedPWM', Int32, queue_size=10)

        rospy.loginfo("Node 'rc_car_base_controller' started. \nListening to 'Steer_max',  param value:  %s " % STEERING_MAX ) 


        rospy.spin()

    except rospy.ROSInterruptException:
        pass
