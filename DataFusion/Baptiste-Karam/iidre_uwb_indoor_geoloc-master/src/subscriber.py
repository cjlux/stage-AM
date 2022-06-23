#!/usr/bin/env python3
import time, sys
import rospy
# from tf import TransformBroadcaster

from std_msgs.msg import String

class iidre_listner:

    def __init__(self, opened_log_file, duration=None, verbose=False):
        self.verbose = verbose
        self.log_file = opened_log_file
        self.subscriber = rospy.Subscriber("/chatter", String, self.callback)
        # self.tfb = TransformBroadcaster()
        self.duration = duration
        self.t0 = time.time()
        print("instance of iidre_listen created...")

    def callback(self, data):
        if self.duration :
        	t = time.time() - self.t0
        	if t > self.duration :
        		sys.exit()
        # self.parsing(data)
        if self.verbose: rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.log_file.write(str(data.data)+"\n")

    def parsing(self, data):
        fb = data.data.split(":")
        fb_cmd = fb[0]
        fb_data = fb[1].split(",")
        time = fb_data[0]

        if fb_cmd == "+DIST":
            # This is usable even if the device has not been preconfigured with the uwbSupervisor
            # Just triangulate the distance (not done here)
            # anchor_id = fb_data[1]
            # anchor_dist = fb_data[2]
            # anchor_xyz = fb_data[3:6]
            # ax_m, ay_m, az_m = map(lambda x: float(x), anchor_xyz)
            data.data = [fb_data[1:6]]

            # if self.publish_anchors:
            #    self.tfb.sendTransform(
            #        (ax_m, ay_m, az_m), (0, 0, 0, 1),   # device position, quaternion
            #        rospy.Time.now(),
            #        anchor_id,
            #        self.device_frame_id)

        elif fb_cmd == "+MPOS":
            # This is usable if device has been preconfigured with the uwbSupervisor
            # x, y, z = fb_data[1:4]
            # Convert from centimeters (in the JSON infra file) to meters
            # x_m, y_m, z_m = map(lambda x: float(x), [x, y, z])
            data.data = [fb_data[1], fb_data[2]]

            # self.tfb.sendTransform(
            #    (x_m, y_m, z_m), (0, 0, 0, 1),   # device position, quaternion
            #    rospy.Time.now(),
            #    self.device_name,
            #    self.device_frame_id)


if __name__ == '__main__':

   import time, sys
   import argparse
   
   parser = argparse.ArgumentParser()
   parser.add_argument('--duration', action="store", dest="duration", type=int, 
   			help="duration of the experimentation in seconds")
   
   args = parser.parse_args()
   duration = None
   if args.duration :
   	duration = args.duration
   	print("duration : ", duration)
         
   uniq_file_name = f"./Data_iidre_{time.strftime('%y-%m-%d_%H-%M-%S', time.localtime())}.txt"
   print(f"writing data in <{uniq_file_name}>")

   with open(uniq_file_name, "w") as f:

        listner = iidre_listner(f, duration)

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('iidre_listener', anonymous = True)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()        
