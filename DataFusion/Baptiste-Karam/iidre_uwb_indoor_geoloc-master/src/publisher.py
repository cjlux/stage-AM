#!/usr/bin/env python3
import time, serial, rospy
#Importé par nous
from std_msgs.msg import String

class UwbXyzPublisher(object):
    '''This class...
    '''
    def __init__(self):
        '''Parameters: None
           Get parameters from ROS_param:
           name: 
           part:str, give the name of the device in /dev, default: "/dev/ttyACM0"
           frame_id: 
        '''
        rospy.init_node("iidre_uwb_xyz_publisher")
        self.serial = None
        self.topic_name = "chatter"
        self.device_name = rospy.get_param("name", "uwb")
        self.device_port = rospy.get_param("port", "/dev/ttyACM0")
        self.publish_anchors = rospy.get_param("publish_anchors", True)

    def connect(self):
        '''Try to connect on the serial link. Write messages in rospy.loginfo.
        '''
        if self.serial is not None:
            try:
                self.serial.close()
            except:
                pass

        rospy.loginfo("Connecting to {}...".format(self.device_port))
        self.serial = serial.Serial(self.device_port)
        rospy.loginfo(f"Connected! Now publishing data from '{self.device_name}'...")

    def run(self):
        '''Impose the data rate (100 Hz) of the loop.
           Enter in an infinite loop to read a line from serial link,
           pre-process the string and calls the publish method.
        '''
        rate = rospy.Rate(100) #100 Hz => 10 ms
        while not rospy.is_shutdown():
            try:
                line = self.serial.readline().decode("ascii")
                line = line.strip()          # remove \n or \t or \r at gebin or end of the str
                line = line.replace(' ','0') # we use 2D configutaion: z value is a space
                #print(f"line:<{line}>")
                self.publish(line)
                rate.sleep()
            except (ValueError, IndexError):
                # Ignore the frame in case of any parsing error
                rospy.loginfo("Error when parsing a frame from serial")
            except serial.serialutil.SerialException:
                rospy.logwarn("Device disconnection, retrying...")
                rospy.sleep(2)
                self.connect()

    def publish(self, line):
        '''Publish the data on the topic
        '''
        # Delete the informations about the velocity
        line=line[:len(line)-4]

        #Partie modifiée par nous
        pub = rospy.Publisher(self.topic_name, String, queue_size=10)
        pub.publish(line)

if __name__ == "__main__":
    node = UwbXyzPublisher()
    node.connect()
    node.run()

