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
import json
index = count(step=1)  # time counter

pub_laser_left = rospy.Publisher("sonar_laser_left", LaserScan, queue_size=100)
pub_laser_center = rospy.Publisher(
    "sonar_laser_center", LaserScan, queue_size=100)
pub_laser_right = rospy.Publisher(
    "sonar_laser_right", LaserScan, queue_size=100)
pub_laser_merged = rospy.Publisher(
    "merged_sonar", LaserScan, queue_size=100
)
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
    sonar_data_left = np.zeros(60)
    sonar_data_center = np.zeros(60)
    sonar_data_right = np.zeros(60)
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        time_end = rospy.Time.now()
        j = 0
        for i in range(0, 60, 3):

            pub_servo.publish(i)
            time.sleep(scan_time)

            # Time = next(index)
            sonar_data_left[j] = data_left
            sonar_data_center[j] = data_center
            sonar_data_right[j] = data_right

            # Публикация объединенного LaserScan
            laser_merged = LaserScan()
            laser_merged.header.stamp = time_end
            laser_merged.header.frame_id = 'sonar'
            laser_merged.angle_min = 0
            laser_merged.angle_max = np.pi * 120 / 180
            laser_merged.angle_increment = np.pi / 180
            laser_merged.time_increment = 0.01
            laser_merged.scan_time = scan_time
            laser_merged.range_min = 0.1
            laser_merged.range_max = 10
            laser_merged.ranges = np.concatenate(
                (sonar_data_right, sonar_data_center, sonar_data_left)).tolist()

            laser_merged.intensities = []
            pub_laser_merged.publish(laser_merged)
            j += 1
        pub_servo.publish(0)
        r.sleep()


rospy.init_node('laser', anonymous=True)
# rospy.Subscriber('/sonar', Float64MultiArray, callback)
rospy.Subscriber('/sonar_center', UInt16, callback_center)
rospy.Subscriber('/sonar_left', UInt16, callback_left)
rospy.Subscriber('sonar_right', UInt16, callback_right)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass
