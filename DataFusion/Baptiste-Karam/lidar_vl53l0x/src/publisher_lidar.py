#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time
import VL53L0X

def publisher_lidar():
    pub = rospy.Publisher('range', String, queue_size=10)
    rospy.init_node('publisher_lidar', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    tof = VL53L0X.VL53L0X()
    while not rospy.is_shutdown():
        tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)

        D = tof.get_distance()
        pub.publish(str(D))
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_lidar()
    except rospy.ROSInterruptException:
        pass
