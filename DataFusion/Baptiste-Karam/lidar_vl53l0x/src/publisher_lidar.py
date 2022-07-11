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
        '''Enter in an infinite loop to read a line from serial link,
           pre-process the string and calls the publish method.
           Nota Bene : The rate is defined by the LiDAR according to the mode that you chose.
        '''
        tof = VL53L0X.VL53L0X()
        while not rospy.is_shutdown():
            try:
                tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)
                self.publish(tof)
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
