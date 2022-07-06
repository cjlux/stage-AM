#!/usr/bin/env python3

import time, sys
import rospy
from std_msgs.msg import String

class LiDAR_listner:
    '''This class allows to get the information published by the publisher of LiDAR
       and stores those data in a file Data_LiDAR__{year}_{month}_{day}_{hour}_{minutes}_{seconds}.txt
       registered in the tree where the code is executed.
    '''
    def __init__(self, opened_log_file, duration=None, verbose=False):
        '''Parameters :
                opened_log_file : the file where the data are stored
                duration : duration of the algorithm's execution
                verbose : define if there will be messages printed in the terminal
           Functions :
           rospy.Subscriber takes in parameters : the topic on which it is registered
           as a subscriber, the given type of the messages and a function to write what it hears.
           The current time is also collected to apply the duration you want.
        '''
        self.verbose = verbose
        self.log_file = opened_log_file
        self.subscriber = rospy.Subscriber("/range", String, self.callback)
        print("instance of LiDAR_listen created...")

    def callback(self, data):
        '''It writes a message in rospy.loginfo about the data
           it hears (when the verbose variable is set to True). Then, it writes the
           data in the file.
        '''

        # self.parsing(data)
        if self.verbose: rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.log_file.write(str(data.data)+"\n")

    def parsing(self, data):
        fb = data.data


if __name__ == '__main__':

   import time, sys

   uniq_file_name = f"./Data_LiDAR_{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        listner = LiDAR_listner(f)
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('LiDAR_listner', anonymous = True)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()