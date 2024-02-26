#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import LaserScan
from queue import Queue
from tf.transformations import euler_from_quaternion
import numpy as np
import time
from itertools import count

index = count(step=1)  # time counter

pub_laser_left = rospy.Publisher("sonar_laser_left", LaserScan, queue_size=100)
pub_laser_center = rospy.Publisher(
    "sonar_laser_center", LaserScan, queue_size=100)
pub_laser_right = rospy.Publisher(
    "sonar_laser_right", LaserScan, queue_size=100)
pub_servo = rospy.Publisher("servo", UInt16, queue_size=10)

data_left = None
data_right = None
data_center = None
# data_queue = Queue(3)


def callback_left(sonars_data):
    global data_left
    data_left = sonars_data.data / 10


def callback_right(sonars_data):
    global data_right
    data_right = sonars_data.data / 10


def callback_center(sonars_data):
    global data_center
    data_center = sonars_data.data / 10



scan_time = 0.1


def laser():
    time_begin = rospy.Time.now()
    r = rospy.Rate(10)  # 10hz
    sonar_data_left = np.zeros(60)
    sonar_data_center = np.zeros(60)
    sonar_data_right = np.zeros(60)
    while not rospy.is_shutdown():
        time_end = rospy.Time.now()


        for i in range(0, 60, 3):
            pub_servo.publish(i)
            time.sleep(scan_time)
            laser_left = LaserScan()
            laser_center = LaserScan()
            laser_right = LaserScan()
            Time=next(index)
        
            sonar_data_left[i] = data_left
            
            laser_left.header.stamp = time_end
            laser_left.header.frame_id = 'sonar'
            laser_left.angle_min = 0
            laser_left.angle_max = np.pi * 90 / 180
            laser_left.angle_increment = np.pi / 180
            laser_left.time_increment = 0.01
            laser_left.scan_time = scan_time
            laser_left.range_min = 0.1
            laser_left.range_max = 10
            laser_left.ranges = sonar_data_left
            laser_left.intensities = []
            
            sonar_data_center[i] = data_center

            laser_center.header.stamp = time_end
            laser_center.header.frame_id = 'sonar'
            laser_center.angle_min = np.pi * 90 / 180
            laser_center.angle_max = np.pi * 180 / 180
            laser_center.angle_increment = np.pi / 180
            laser_center.time_increment = 0.01
            laser_center.scan_time = scan_time
            laser_center.range_min = 0.1
            laser_center.range_max = 10
            laser_center.ranges = sonar_data_center
            laser_center.intensities = []
            
            sonar_data_right[i] = data_right

            laser_right.header.stamp = time_end
            laser_right.header.frame_id = 'sonar'
            laser_right.angle_min = np.pi * 180 / 180
            laser_right.angle_max = np.pi * 270 / 180
            laser_right.angle_increment = np.pi / 180
            laser_right.time_increment = 0.01
            laser_right.scan_time = scan_time
            laser_right.range_min = 0.1
            laser_right.range_max = 10
            laser_right.ranges = sonar_data_right
            laser_right.intensities = []

            pub_laser_left.publish(laser_left)
            pub_laser_center.publish(laser_center)
            pub_laser_right.publish(laser_right)

    if i == 180:
        pub_servo.publish(0)
    r.sleep()


rospy.init_node('laser', anonymous=True)
# rospy.Subscriber('/sonar', Float64MultiArray, callback)
rospy.Subscriber('/sonar_center', Float64, callback_left)
rospy.Subscriber('/sonar_left', Float64, callback_center)
rospy.Subscriber('sonar_right', Float64, callback_right)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass
