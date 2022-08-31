#!/usr/bin/env python3

from geometry_msgs.msg import QuaternionStamped
from math import cos, sin
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
import numpy as np
import rospy, message_filters, quaternion, time, sys, argparse

class miniapterros_listener:
    ''' 
    This class allows to get the information published by the publishers of IIDRE, LiDAR and MTi-30
    and stores those data in a file Data_fusion_{year}_{month}_{day}_{hour}_{minutes}_{seconds}.txt
    registered in the tree where the code is executed.
    '''
    def __init__(self, log_file, verbose=False):
        '''
        Parameters :
          log_file : the file where the data are stored
          verbose : define if there will be messages printed in the terminal
        Functions :
          message_filters.Subscriber takes in parameters : the topic on which it is registered
          as a subscriber and the given type of the messages.

          message_filters.ApproximateTimeSynchronizer() synchronizes messages by their timestamp,
          only pass them through when all have arrived

          Parameters :
              a list of multiple subscribers,
              queue size : sets of messages should be stored from each input filter
                           (by timestamp) while waiting for all messages to arrive,
              slop : delay in secondes with which messages can be synchronized, 
              allow_headerless : allow the access to different headers in order 
                                 to have the TimeStamped for each sensor.
        '''
        self.verbose = verbose
        self.log_file = log_file

        self.subscriber_iidre = message_filters.Subscriber("/iidre_position", String)
        self.subscriber_lidar = message_filters.Subscriber("/lidar_height", String)
        self.subscriber_mti = message_filters.Subscriber("/filter/quaternion", QuaternionStamped)
        print("instance of MiniApterros created ...")
        

        self.tss = message_filters.ApproximateTimeSynchronizer([self.subscriber_iidre, 
                                                                self.subscriber_lidar, 
                                                                self.subscriber_mti],
                                                                queue_size = 10, 
                                                                slop = 1, 
                                                                allow_headerless=True)
        
        
        self.tss.registerCallback(self.callback)


    def callback(self, data_iidre, data_lidar, data_mti):
        '''
        It calls 3 parsing() function for the sensors to keep just data we need.
        It contains a conversion function to get euler angles from quaternion.
        Then, it  writes a message in rospy.loginfo about the data
        it hears (when the verbose variable is set to True).
        Then, it writes the data in the file in blocs :
             Time
             IIDRE_DATA
             LiDAR_DATA
             MTi_DATA (quaternion and Euler angles)

             New_treated_coordinates (using quaternion)
             New_treated_coordinates (using angles of Euler)
        '''
        
        if self.log_file.closed == False:
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
                
                # Writing in file

                self.log_file.write("Time-MTi-30: "+str(data_mti.header)+"\n")
                self.log_file.write("Time-LiDAR: "+str(data_lidar.data[0])+"\n")
                self.log_file.write("DATA_IIDRE:"+ str(data_iidre.data[0])+","+
                                                   str(data_iidre.data[1])+"\n")
                self.log_file.write("DATA_LiDAR:"+ str(data_lidar.data[1])+"\n")
                self.log_file.write("DATA_MTi-30 - quaternion:"+str(data_mti.quaternion[0])+","+
                                                                str(data_mti.quaternion[1])+","+
                                                                str(data_mti.quaternion[2])+","+
                                                                str(data_mti.quaternion[3])+"\n")
                self.log_file.write("DATA_MTi-30 - euler:"+str(data_mti_euler[0])+","+
                                                           str(data_mti_euler[1])+","+
                                                           str(data_mti_euler[2])+"\n")

                self.log_file.write("Nouvelles coordonnées - quaternion:"+str(data_iidre.data[0])+","+
                                                                          str(data_iidre.data[1])+","+
                                                                          str(abs(height_quaternion))+"\n")
                self.log_file.write("Nouvelles coordonnées - euler:"+str(data_iidre.data[0])+","+
                                                                     str(data_iidre.data[1])+","+
                                                                     str(matrix_new[2,0])+"\n"+"\n")
        else:
                return

    def parsing_iidre(self, data_iidre):
        ''' 
        This enables to only take the relevant information of the message it hears.
        So, it splits the information at each ':' to first see if the information is about
        the distance between each anchor and the tag or the position of the tag.
        Then, it reduces the size of the data to only write in the file the information we want.
        '''
        fb = data_iidre.data.split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].split(",")

        data_iidre.data = [fb_data[1], fb_data[2]]

    def parsing_lidar(self, data_lidar):
        ''' 
        Splits data at each ',' and takes the second part of data
        which corresponds to the distance measured by LiDAR
        '''
        fb = data_lidar.data.split(",")
        data_lidar.data = fb[1]
        data_lidar.data = int(float(data_lidar.data))

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
        data_mti.quaternion = [fb_data_mti[1],fb_data_mti[3],fb_data_mti[5],fb_data_mti[7]]

if __name__ == '__main__':

   parser = argparse.ArgumentParser()
   parser.add_argument("--file_prefix", type=str, default="")
   parser.add_argument("--duration", type=float, default=0.0)
   args = parser.parse_args()
   file_prefix = args.file_prefix
   duration = args.duration

   uniq_file_name = f"./Data_fusion_"
   if file_prefix != "":
      uniq_file_name += f"{file_prefix}_"
   uniq_file_name += f"{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        rospy.init_node('miniapterros_listener', anonymous = True)
        listener = miniapterros_listener(f)

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
