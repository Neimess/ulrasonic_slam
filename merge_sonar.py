import rospy
from sensor_msgs.msg import LaserScan





if __name__ == '__main__':
    try:
        laser_scan_merger = LaserScanMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
