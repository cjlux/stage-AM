#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped

class xsens_mti_listener:

    def __init__(self, opened_log_file, verbose=False):
        self.verbose = verbose
        self.log_file = opened_log_file
        self.subscriber = rospy.Subscriber("/imu/acceleration", Vector3Stamped, self.callback)
        print("instance of xsens_mti_listen created...")

    def callback(self, data):
        self.parsing(data)
        if self.verbose: rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.vector))
        self.log_file.write(str(data.vector[0])+","+str(data.vector[1])+","+str(data.vector[2])+"\n")

    def parsing(self, data):
        data.vector = str(data.vector).replace("\n", ":")       # Replace the \n by : in the str
        fb_data = data.vector.split(":")                        # Récupère les accelerations selon x, y et z

        data.vector = [fb_data[1],fb_data[3],fb_data[5]]

if __name__ == '__main__':

   with open("./Data_mti_acceleration.txt", "w") as f:

        listner = xsens_mti_listener(f)

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('xsens_mti_listener', anonymous = True)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
