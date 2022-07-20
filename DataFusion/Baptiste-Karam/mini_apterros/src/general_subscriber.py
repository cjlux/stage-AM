#!/usr/bin/env python3

import time, rospy
from std_msgs.msg import String
from geometry_msgs.msg import QuaternionStamped
import message_filters
from scipy.spatial.transform import Rotation
from math import radians, cos

class miniapterros_listner:
    ''' This class allows to get the information published by the publishers of IIDRE, LiDAR and MTi-30
        and stores those data in a file Data_fusion_{year}_{month}_{day}_{hour}_{minutes}_{seconds}.txt
        registered in the tree where the code is executed.
    '''
    def __init__(self, log_file, verbose=False):
        '''Parameters :
                log_file : the file where the data are stored
                verbose : define if there will be messages printed in the terminal
           Functions :
                message_filters.Subscriber takes in parameters : the topic on which it is registered
                as a subscriber and the given type of the messages.
                
                message_filters.ApproximateTimeSynchronizer() takes in parameters : a list of multiple subscribers,
                queue size, time interval for time synchronization and a parameter that allow the access to different 
                headers in order to have the TimeStamped for each sensor.
        '''
        self.verbose = verbose
        self.log_file = log_file
        
        self.subscriber_iidre = message_filters.Subscriber("/iidre_position", String)
        self.subscriber_lidar = message_filters.Subscriber("/lidar_height", String)
        self.subscriber_mti = message_filters.Subscriber("/filter/quaternion", QuaternionStamped)
        print("instance of MiniApterros created ...")

        self.tss = message_filters.ApproximateTimeSynchronizer([self.subscriber_iidre, self.subscriber_lidar, self.subscriber_mti],queue_size = 10, slop = 1, allow_headerless=True)
        self.tss.registerCallback(self.callback)
        

    def callback(self, data_iidre, data_lidar, data_mti):
        '''It calls 3 parsing() function for the sensors to keep just data we need.
           It contains a conversion function to get euler angles from quaternion. 
           Then, it  writes a message in rospy.loginfo about the data
           it hears (when the verbose variable is set to True). 
           Then, it writes the data in the file in blocs :
                Time
                IIDRE_DATA
                LiDAR_DATA
                MTi_DATA
        '''
        self.parsing_iidre(data_iidre)
        self.parsing_mti(data_mti)
        self.parsing_lidar(data_lidar, data_mti)

        if self.verbose:
            rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s", str(data_iidre.data), str(data_lidar.data), str(data_mti.quaternion))

        self.log_file.write("Time: "+str(data_mti.header)+"\n")
        self.log_file.write("DATA_IIDRE :"+ str(data_iidre.data[0])+","+str(data_iidre.data[1])+"\n")
        self.log_file.write("DATA_LiDAR :"+ str(data_lidar.data)+"\n")
        self.log_file.write("DATA_MTi-30 :"+str(radians(data_mti.quaternion[0]))+","+str(radians(data_mti.quaternion[1]))+","+str(radians(data_mti.quaternion[2]))+"\n"+"\n")

    def parsing_iidre(self, data_iidre):
        '''This enables to only take the relevant information of the message it hears.
           So, it splits the information at each ':' to first see if the information is about
           the distance between each anchor and the tag or the position of the tag.
           Then, it reduces the size of the data to only write in the file the information we want.
        '''
        fb = data_iidre.data.split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].split(",")
        time = fb_data[0]

        if fb_cmd == "+MPOS":
            # This is usable if device has been preconfigured with the uwbSupervisor
            # Convert from centimeters (in the JSON infra file) to meters
            data_iidre.data = [fb_data[1], fb_data[2]]

    def parsing_lidar(self, data_lidar, data_mti):
        '''It splits data at each ',' and takes the second part of data
           which corresponds to the distance measured by LiDAR and corrected
           by the angle calculate by the MTi-30
        '''
        fb = data_lidar.data.split(",")
        data_lidar.data = fb[1]
        data_lidar.data = int(data_lidar.data) * cos(radians(data_mti.quaternion[0])) * cos(radians(data_mti.quaternion[1]))

    def parsing_mti(self, data_mti):
        ''' This function receives data as argument.
            It splits data to get only time and the quaternion
        '''
        data_mti.header = str(data_mti.header).replace("\n", ":")
        tmp_data_mti = data_mti.header.split(":")
        data_mti.header = float(tmp_data_mti[5]) + int(tmp_data_mti[7]) * 1e-9

        data_mti.quaternion = str(data_mti.quaternion).replace("\n", ":")       # Replace the \n by : in the str
        fb_data_mti = data_mti.quaternion.split(":")                            # Récupère les accelerations selon x, y et z
        data_mti.quaternion = [fb_data_mti[1],fb_data_mti[3],fb_data_mti[5],fb_data_mti[7]]
        
        # Transformation quaternion to Euler
        data_mti.quaternion = Rotation.from_quat(data_mti.quaternion)
        data_mti.quaternion = data_mti.quaternion.as_euler('xyz', degrees=True)
        
if __name__ == '__main__':

   uniq_file_name = f"./Data_fusion_{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        rospy.init_node('miniapterros_listner', anonymous = True)
        listner = miniapterros_listner(f)

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
