#!/usr/bin/env python3

import time, sys, argparse
import rospy
from std_msgs.msg import String

class iidre_listner:
    '''
    This class allows to get the information published by the publisher of IIDRE
    and stores those data in a file Data_iidre_{year}_{month}_{day}_{hour}_{minutes}_{seconds}.txt
    registered in the directory where the code is executed.
    '''
    def __init__(self, opened_log_file, verbose=False):
        '''
        Parameters :
          opened_log_file : the file where the data are stored
          verbose : define if there will be messages printed in the terminal
        
        Functions :
        rospy.Subscriber takes in parameters : the topic on which it is registered
        as a subscriber, the given type of the messages and a function to write what it hears.
        '''
        self.verbose = verbose
        self.log_file = opened_log_file
        self.subscriber = rospy.Subscriber("/iidre_position", String, self.callback)
        print("instance of 'iidre_listner' created...")

    def callback(self, data):
        '''
        Calls the method 'parsing' to filter the relevant information in the data of the topic. 
        If verbose, writes a message in rospy.loginfo about the data it hears.
        Finaly writes the data in the file.
        '''
        self.parsing(data)
        if self.verbose: rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.log_file.write(data.data+"\n")

    def parsing(self, data):
        '''
        Filter the data to take only the relevant information of the message it hears.
        Splits the information at each ':' to first see when the information is about
        the position of the tag.
        Then, it reduces the size of the data to only write in the file the information we want.
        '''
        fb = data.data.split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].split(",")
        time = fb_data[0]

        if fb_cmd == "+MPOS":
            # This is usable if device has been preconfigured with the uwbSupervisor
            # Convert from centimeters (in the JSON infra file) to meters
            data.data = f"{time},{fb_data[1]},{fb_data[2]}"


if __name__ == '__main__':

   import time, sys

   parser = argparse.ArgumentParser()
   parser.add_argument("--file_prefix", type=str, default="")
   parser.add_argument("--duration", type=int, default=0)
   args = parser.parse_args()
   file_prefix = args.file_prefix
   duration = args.duration

   uniq_file_name = f"./Data_iidre_"
   if file_prefix != "":
      uniq_file_name += f"{file_prefix}_"
   uniq_file_name += f"{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        listner = iidre_listner(f)

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('iidre_listener', anonymous = True)

        # spin() simply keeps python from exiting until this node is stopped
        if duration:
            rospy.sleep(duration)
        else:
            rospy.spin()
        
