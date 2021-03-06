#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped

class xsens_mti_listener:
    '''
       This class registers what come from a specified topic.
       It displays the 3 components of linear acceleration
    '''

    def __init__(self, opened_log_file, verbose=False):
        '''Parameters :
           Opened_log_file : the file where data is stored
           Verbose         : define if there will be messages printed in the terminal
           Get parameters from ROS_param:
           subscriber : register messages that came from topic "/imu/acceleration",
                        vector3Stamped is the type of data received,
                        callback() calls data when it is received.
        '''
        self.verbose = verbose
        self.log_file = opened_log_file
        self.subscriber = rospy.Subscriber("/imu/acceleration", Vector3Stamped, self.callback)
        print("instance of xsens_mti_listen created...")

    def callback(self, data):
        ''' This function calls data when it is received.
            It calls the parsing() function,
            Then it displays time and the 3 components of linear acceleration when verbose=True
            Parameters:
            Data: data received
        '''
        self.parsing(data)
        if self.verbose:
          rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.vector))
        self.log_file.write("Time: "+str(data.header)+", Acceleration: "+str(data.vector[0])+","+str(data.vector[1])+","+str(data.vector[2])+"\n")

    def parsing(self, data):
        ''' This function receives data as argument.
            It splits data to get only the timestamp and linear acceleration
        '''
        data.header = str(data.header).replace("\n", ":")
        tmp_data = data.header.split(":")
        data.header = float(tmp_data[5]) + int(tmp_data[7]) * 1e-9

        data.vector = str(data.vector).replace("\n", ":")       # Replace the \n by : in the str
        fb_data = data.vector.split(":")                        # R??cup??re les accelerations selon x, y et z

        data.vector = [fb_data[1],fb_data[3],fb_data[5]]

if __name__ == '__main__':

   import time

   uniq_file_name = f"./Data_MTi_acceleration_{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        listner = xsens_mti_listener(f)

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('xsens_mti_listener', anonymous = True)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
