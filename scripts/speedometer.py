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

    # Filter speed estimate
    global exp_speed_est
    global current_est_speed
    speed_estimates.append(current_speed)
    #rospy.loginfo(" estimate  %s", str(len(speed_estimates)))

    if (len(speed_estimates) > 20):
        exp_speed_est = ExpMovingAverage(speed_estimates, 10)
        current_est_speed = float(exp_speed_est[-1])
        speed_estimates.pop(0)
        exp_speed_est = np.delete(exp_speed_est, 1)

    pub_speed_filtered.publish(current_est_speed)



    #rospy.loginfo("Exp speed estimate  %s", str(exp_speed_est))

if __name__ == '__main__':
    try:

        rospy.init_node('rc_car_speedometer')
        
	rospy.Subscriber('/fix', NavSatFix, gps_callback, queue_size=10)

        pub_speed_filtered = rospy.Publisher('/rc_car/speed_filtered/new', Float32, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
               
