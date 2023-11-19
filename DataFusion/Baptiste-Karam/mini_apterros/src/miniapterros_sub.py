#!/usr/bin/env python3

#
# 2023-06-13 - JLC : convert mti.quaternion components to flost
#

from geometry_msgs.msg import QuaternionStamped
from math import cos, sin
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
import numpy as np
import rospy, message_filters, quaternion, time, sys, argparse

class miniapterros_listener:
    '''
    This class gets the topics published by tIIDRE, LiDAR and MTi-30 and write the dictionnary of the
    data the /MiniAPTERROS  ROS parameter
    '''
    def __init__(self, verbose=False):
        '''
        Parameters:
          verbose: define if there will be messages printed in the terminal

	Functions used:
          message_filters.Subscriber() 
              takes in parameters the topic on which it is registered as a subscriber and the given 
              type of the messages.

          message_filters.ApproximateTimeSynchronizer() 
              synchronizes messages by their timestamp, only pass them through when all have arrived

              Parameters:
                a list of multiple subscribers,
                queue size: sets of messages should be stored from each input filter
                          (by timestamp) while waiting for all messages to arrive,
                slop: delay in secondes with which messages can be synchronized,
                allow_headerless: allow the access to different headers in order
                                 to have the TimeStamped for each sensor.
        '''
        self.verbose = verbose

        self.subscriber_iidre = message_filters.Subscriber("/iidre_position", String)
        self.subscriber_lidar = message_filters.Subscriber("/lidar_height", String)
        self.subscriber_mti   = message_filters.Subscriber("/filter/quaternion", QuaternionStamped)
        print("instances of MiniApterros subscribers created ...")

        self.tss = message_filters.ApproximateTimeSynchronizer([self.subscriber_iidre,
                                                                self.subscriber_lidar,
                                                                self.subscriber_mti],
                                                                queue_size = 10,
                                                                slop = 1,
                                                                allow_headerless=True)

        self.tss.registerCallback(self.callback)


    def callback(self, data_iidre, data_lidar, data_mti):
        '''
        Calls the 3 "parsing" functions for the sensors to filter out the needed data.
        It contains a conversion function to get euler angles from quaternion.
        Then, it  writes the dictionnary of the data as the /MiniAPTERROS ROS parameter.
        '''

        self.parsing_iidre(data_iidre)
        self.parsing_mti(data_mti)
        self.parsing_lidar(data_lidar)

        if self.verbose:
            rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s",
                                                  str(data_iidre.data),
                                                  str(data_lidar.data),
                                                  str(data_mti.quaternion))

        # Fusion of LiDAR and IMU data

        # Quaternion
        vector_u = np.quaternion(0.0, 0.0, 0.0, float(data_lidar.data[1]))
        quaternion = np.quaternion(float(data_mti.quaternion[3]),
                                   float(data_mti.quaternion[0]),
                                   float(data_mti.quaternion[1]),
                                   float(data_mti.quaternion[2]))
        vector_v = quaternion.conjugate()*vector_u*quaternion
        height_quaternion = vector_v.z    # Retrieve the last component of the quaternion

        # Euler
        # Transformation quaternion to Euler
        data_mti_euler = []
        data_mti_euler = Rotation.from_quat(data_mti.quaternion)
        data_mti_euler = data_mti_euler.as_euler('xyz')
        data_mti_euler = data_mti_euler.tolist()
        roll = abs(data_mti_euler[0])
        pitch = abs(data_mti_euler[1])
        yaw = abs(data_mti_euler[2])

        matrix_xyz = np.matrix([[0], [0], [data_lidar.data[1]]])

        matrix_roll = [[1, 0, 0],[0, cos(roll), -sin(roll)],[0, sin(roll), cos(roll)]]
        matrix_pitch = [[cos(pitch), 0, sin(pitch)],[0, 1, 0],[-sin(pitch), 0, cos(pitch)]]
        matrix_yaw = [[cos(yaw), -sin(yaw), 0],[sin(yaw), cos(yaw), 0], [0, 0, 1]]

        matrix_euler = np.dot(matrix_yaw, matrix_pitch)
        matrix_euler = np.dot(matrix_euler, matrix_roll)
        matrix_new = np.dot(matrix_euler, matrix_xyz)

        # Writing the dictionnary of data as a ROS parameter:

        data = dict()
        data["MTI30"] = [data_mti.header, data_mti.quaternion, data_mti_euler]
        data["IIDRE"] = data_iidre.data
        data["LIDAR"] = data_lidar.data


        rospy.set_param('MiniAPTERROS', data)
        
        return

    def parsing_iidre(self, data_iidre):
        '''
        This enables to only take the relevant information of the message it hears.
        So, it splits the information at each ':' to first see if the information is about
        the distance between each anchor and the tag or the position of the tag.
        Then, it reduces the size of the data to only write in the file the information we want.
        '''
        line = data_iidre.data.split(";")
        Time = line[0].split(":")[1]
        fb = line[1].split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].split(",")

        data_iidre.data = [float(Time), float(fb_data[1]), float(fb_data[2])]

    def parsing_lidar(self, data_lidar):
        '''
        Splits data at each ',' and takes the second part of data
        which corresponds to the distance measured by LiDAR
        '''
        fb = data_lidar.data.split(",")
        data_lidar.data = [float(fb[0]), round(float(fb[1]))]

    def parsing_mti(self, data_mti):
        '''
        Receives data as argument: splits data to get only time and the quaternions.
        '''
        # concatenate all the lines:
        data_mti.header = str(data_mti.header).replace("\n", ":")
        tmp_data_mti = data_mti.header.split(":")
        data_mti.header = float(tmp_data_mti[5]) + int(tmp_data_mti[7]) * 1e-9

        data_mti.quaternion = str(data_mti.quaternion).replace("\n", ":")       # Replace the \n by : in the str
        fb_data_mti = data_mti.quaternion.split(":")                            # Retrieve the quaternions Roll, Pitch and Yaw
        data_mti.quaternion = [float(fb_data_mti[1]),float(fb_data_mti[3]),float(fb_data_mti[5]),float(fb_data_mti[7])]

if __name__ == '__main__':

   parser = argparse.ArgumentParser()
   parser.add_argument("--duration", type=float, default=0.0)
   args = parser.parse_args()
   duration = args.duration
   rospy.init_node('miniapterros_listener', anonymous = True)
   listener = miniapterros_listener(False)

   # In ROS, nodes are uniquely named. If two nodes with the same
   # name are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.

   # spin() simply keeps python from exiting until this node is stopped
   if duration:
       rospy.sleep(duration)
   else:
       rospy.spin()

