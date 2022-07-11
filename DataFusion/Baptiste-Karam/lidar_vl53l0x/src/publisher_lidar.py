#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
import VL53L0X

class LiDAR_publisher(object):
    '''This class allows to publish the data got by LiDAR on a specific topic.
    '''
    def __init__(self):
        '''Parameters: None
           Get parameters from ROS_param:
           name : str, give the name of the device, default: "LiDAR"
           port : str, give the name of the device in /dev, default: "GPiO"
           Initialization of the topic by the Publisher :
           The name of the topic and the type of messages allowed
        '''
        rospy.init_node("publisher_lidar")
        self.serial = None
        self.topic_name = rospy.Publisher('range', String, queue_size=10)

    def run(self):
        '''Impose the data rate (50 Hz here) of the loop.
           Enter in an infinite loop to read a line from serial link,
           pre-process the string and calls the publish method.
        '''
        rate = rospy.Rate(50) #50 Hz => 20 ms
        tof = VL53L0X.VL53L0X()
        while not rospy.is_shutdown():
            try:
                tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
                self.publish(tof)
                rate.sleep()
            except (ValueError, IndexError):
                # Ignore the frame in case of any parsing error
                rospy.loginfo("Error when parsing a frame from serial")
            except serial.serialutil.SerialException:
                rospy.logwarn("Device disconnection, retrying...")
                rospy.sleep(2)
                self.connect()

    def publish(self, tof):
        '''Publish the data on the topic
        '''
        self.topic_name.publish("{},{}".format(rospy.get_time(), tof.get_distance()))

if __name__ == '__main__':
    node = LiDAR_publisher()
    node.run()
