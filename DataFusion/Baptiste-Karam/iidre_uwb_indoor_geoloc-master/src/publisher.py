#!/usr/bin/env python3
import time, serial, rospy
#Importé par nous
from std_msgs.msg import String

class UwbXyzPublisher(object):
    '''
    This class allows to publish the data from the IIDRE system on a specific topic.
    '''
    def __init__(self):
        '''
        Parameters: None
        
        Parameters taken from ROS_param:
          name:str: the name of the device, default: "uwb"
          port:str: the name of the serial line, default: "/dev/ttyACM0"
           
        Initialization of the topic by the Publisher:
        The name of the topic and the type of messages allowed.
        '''
        rospy.init_node("iidre_uwb_xyz_publisher")
        self.serial      = None   # will be set by the 'connect' mthod.
        self.publisher   = rospy.Publisher('iidre_position', String, queue_size=10)
        self.device_name = rospy.get_param("name", "uwb")
        self.device_port = rospy.get_param("port", "/dev/ttyACM0")
        
        '''
        JLC: from https://wiki.ros.org/rospy/Overview/Publishers%20and%20SubscribersSetting:
        the queue_size to 1 is a valid approach if you want to make sure that a new published value 
        will always prevent any older not yet sent values to be dropped. This is good for, say, 
        a sensor that only cares about the latest measurement. e.g. never send older measurements 
        if a newer one exists. '''

    def connect(self):
        '''
        Try to connect on the serial link. Write messages in rospy.loginfo.
        '''
        if self.serial is not None:
            try:
                self.serial.close()
            except:
                pass

        rospy.loginfo(f"Connecting to {self.device_port}...")
        self.serial = serial.Serial(self.device_port)
        rospy.loginfo(f"Connected! Now publishing data from '{self.device_name}'...")

    def run(self):
        '''
        Impose the data rate (100 Hz) of the loop.
        Enter in an infinite loop to read a data line from the serial link,
        pre-process the string and calls the publish method.
        '''
        rate = rospy.Rate(100)      #100 Hz => 10 ms
        while not rospy.is_shutdown():
            try:
                line = self.serial.readline().decode("ascii")
                line = line.strip()          # remove \n or \t or \r at begin or end of the str
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
        '''
        Publish the data on the topic
        '''
        # Delete the informations about the velocity
        line=line[:len(line)-4]

        self.publisher.publish(line)

if __name__ == "__main__":
    node = UwbXyzPublisher()
    node.connect()
    node.run()
    
