#!/usr/bin/env python

import rospy
import serial
import argparse
from std_msgs.msg import String
import VL53L0X

class LiDAR_publisher(object):
    '''This class allows to publish the data got by LiDAR on a specific topic.
    '''

    LIDAR_MODE =  {'GOOD_ACCURACY': VL53L0X.VL53L0X_GOOD_ACCURACY_MODE, 
                   'BETTER_ACCURACY': VL53L0X.VL53L0X_BETTER_ACCURACY_MODE, 
                   'BEST_ACCURACY': VL53L0X.VL53L0X_BEST_ACCURACY_MODE,
                   'LONG_RANGE': VL53L0X.VL53L0X_HIGH_SPEED_MODE,
                   'HIGH_SPEED': VL53L0X.VL53L0X_HIGH_SPEED_MODE}
    
    def __init__(self, mode):
        '''Parameters:
             mode:str: the lidar mode, see allowed values in LIDAR_MODE.
        '''
        rospy.init_node("publisher_lidar")
        self.mode   = mode
        self.topic_name = rospy.Publisher('range', String, queue_size=10)

    def run(self):
        '''Enter in an infinite loop to read data on the I2C bus,
           pre-process the string and calls the publish method.
           Nota Bene : The data rate imposed by the LiDAR mode.
        '''
        tof = VL53L0X.VL53L0X()
        while not rospy.is_shutdown():
            try:
                tof.start_ranging(LiDAR_publisher.LIDAR_MODE[self.mode])
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
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar_mode", type=str, default="",
                        help="Lidar mode in ('GOOD_ACCURACY', 'BETTER_ACCURACY', 'BEST_ACCURACY', 'LONG_RANGE', 'HIGH_SPEED')")
    args = parser.parse_args()
    lidar_mode = args.lidar_mode
    lidar_mode = lidar_mode.upper()
    
    if lidar_mode in LiDAR_publisher.LIDAR_MODE.keys():
        node = LiDAR_publisher(lidar_mode)
        node.run()
    else:
        print(f"Error: mode <{lidar_mode}> unknown.\nUse one of {LiDAR_publisher.LIDAR_MODE.keys()}.")

