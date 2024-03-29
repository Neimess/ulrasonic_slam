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
import math
index = count(step=1)  # time counter

pub_laser_left = rospy.Publisher("sonar_laser", LaserScan, queue_size=100)
pub_servo = rospy.Publisher("servo", UInt16, queue_size=10)

data_left = None
data_right = None
data_center = None
# data_queue = Queue(3)

left_data = np.zeros(180)
right_data = np.zeros(180)
center_data = np.zeros(180)


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
    combined_ranges = np.zeros(180)

    while not rospy.is_shutdown():
        time_end = rospy.Time.now()

        for i in range(0, 160, 1):
            print(data_left, data_center, data_right)
            combined_ranges[i] = data_left
            combined_ranges[i] = data_center
            combined_ranges[i] = data_right
            lidar_msg = LaserScan()
            lidar_msg.header.stamp = rospy.Time.now()
            lidar_msg.header.frame_id = "lidar"
            # Минимальный угол сканирования (90 градусов влево)
            lidar_msg.angle_min = -math.pi / 2
            # Максимальный угол сканирования (90 градусов вправо)
            lidar_msg.angle_max = math.pi / 2
            # Инкремент угла сканирования (1 градус)
            lidar_msg.angle_increment = math.pi / 180.0
            lidar_msg.time_increment = 0.0
            lidar_msg.scan_time = 0.1
            lidar_msg.range_min = 0.1
            lidar_msg.range_max = 10.0
            lidar_msg.ranges = combined_ranges
            lidar_msg.intensities = []
            pub_laser_left.publish(lidar_msg)

    if i == 180:
        pub_servo.publish(0)
    r.sleep()


rospy.init_node('laser', anonymous=True)
# rospy.Subscriber('/sonar', Float64MultiArray, callback)
rospy.Subscriber('/sonar_center', UInt16, callback_left)
rospy.Subscriber('/sonar_left', UInt16, callback_center)
rospy.Subscriber('sonar_right', UInt16, callback_right)

if __name__ == '__main__':
    try:
        laser()
    except rospy.ROSInterruptException:
        pass
