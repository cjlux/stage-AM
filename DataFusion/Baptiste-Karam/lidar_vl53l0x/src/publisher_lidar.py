#!/usr/bin/env python3

import rospy
import serial
import argparse
from std_msgs.msg import String

import time
import board
import busio
import adafruit_vl53l0x

class LiDAR_publisher(object):
    '''This class allows to publish the data got by LiDAR on a specific topic.
    '''

    LIDAR_MODE =  {'GOOD_ACCURACY': 33000,
                   'BEST_ACCURACY': 200000,
                   'HIGH_SPEED': 20000}

    def __init__(self, mode):
        '''Parameters:
             mode:str: the lidar mode, see allowed values in LIDAR_MODE.
        '''
        rospy.init_node("publisher_lidar")
        self.mode = mode
        self.topic_name = rospy.Publisher('range', String, queue_size=10)

    def run(self):
        '''Enter in an infinite loop to read data on the I2C bus,
           pre-process the string and calls the publish method.
           Nota Bene : The data rate imposed by the LiDAR mode.
        '''
        # Initialize I2C bus and sensor.
        i2c = busio.I2C(board.SCL, board.SDA)
        tof = adafruit_vl53l0x.VL53L0X(i2c)

        while not rospy.is_shutdown():
            try:
                tof.measurement_timing_budget = LiDAR_publisher.LIDAR_MODE[self.mode]
                self.publish(tof)
            except (ValueError, IndexError):
                # Ignore the frame in case of any parsing error
                rospy.loginfo("Error when parsing a frame from serial")

    def publish(self, tof):
        '''Publish the data on the topic
        '''
        self.topic_name.publish("{},{}".format(rospy.get_time(), tof.range))


if __name__ == '__main__':

    # Old Version :
    
    # parser = argparse.ArgumentParser()
    # parser = parser.replace(":="," ")
    # parser.add_argument("--lidar_mode", "--lidar_mode:=", type=str, default="",
    #                     help="Lidar mode in ('GOOD_ACCURACY', 'BEST_ACCURACY', 'HIGH_SPEED'), to be set in publish_lidar.launch")
    # args = parser.parse_args()
    # lidar_mode = args.lidar_mode
    # lidar_mode = lidar_mode.upper()
    
    # New Version :    
    
    lidar_mode = rospy.get_param('publisher_lidar/lidar_mode')

    if lidar_mode in LiDAR_publisher.LIDAR_MODE.keys():
        node = LiDAR_publisher(lidar_mode)
        node.run()
    else:
        print("Error: mode <{lidar_mode}> unknown.\nUse one of ", format(LiDAR_publisher.LIDAR_MODE.keys()))

