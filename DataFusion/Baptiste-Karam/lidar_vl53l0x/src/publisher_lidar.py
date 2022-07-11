#!/usr/bin/env python

import rospy
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
        self.device_name = rospy.get_param("name", "LiDAR")
        self.device_port = rospy.get_param("port", "GPiO")

    def connect(self):
        '''Try to connect on the serial link. Write messages in rospy.loginfo.
        '''
        if self.serial is not None:
            try:
                self.serial.close()
            except:
                pass

        rospy.loginfo("Connecting to {}...".format(self.device_port))
        self.serial = serial.Serial(self.device_port)
        rospy.loginfo(f"Connected! Now publishing data from '{self.device_name}'...")

    def run(self):
        '''Impose the data rate (50 Hz here) of the loop.
           Enter in an infinite loop to read a line from serial link,
           pre-process the string and calls the publish method.
        '''
        rate = rospy.Rate(50) #50 Hz => 20 ms
        while not rospy.is_shutdown():
            try:
                tof = VL53L0X.VL53L0X()
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

    def publisher_lidar(self, tof):
        '''Publish the data on the topic
        '''
        self.topic_name.publish(f"{rospy.get_time()},{tof.get_distance()}")

if __name__ == '__main__':
    node = LiDAR_publisher()
    node.connect()
    node.run()
