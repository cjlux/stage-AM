#!/usr/bin/env python3

from geometry_msgs.msg import QuaternionStamped
from math import cos, sin
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
import numpy as np
import rospy, message_filters, quaternion, time, sys, argparse

class KalmanFilter(object):
    '''
    This class provides the first Kalman filter estimates and those recalibrated
    using measurements from the IIDRE, LiDAR and MTi-30 sensors.
    '''
    def __init__(self, n_dim, variance, estimate_variance):
        # intial parameters
        self.n_dim = n_dim
        self.sz = (2,self.n_dim)           # size of array
        self.x = np.zeros(self.n_dim)      # truth value (typo in example at top of p. 13 calls this z)

        self.Q = np.array([[variance],
                           [variance],
                           [variance]])    # process variance

        # allocate space for arrays
        self.xhat = np.zeros(self.sz)      # a posteriori estimate of x
        self.P = np.zeros(self.sz)         # a posteriori error estimate
        self.xhatminus = np.zeros(self.sz) # a priori estimate of x
        self.Pminus = np.zeros(self.sz)    # a priori error estimate
        self.K = np.zeros(self.sz)         # gain or blending factor

        self.R = np.array([[estimate_variance],
                           [estimate_variance],
                           [estimate_variance]]) # estimate of measurement variance

        # intial guesses
        self.xhat[0] = np.zeros((1,self.n_dim))
        self.P[0] = np.ones((1,self.n_dim))

    def kalman_estimation(self, measurements) :
        # Update values
        self.xhat[0][:] = self.xhat[1][:]
        self.P[0][:] = self.P[1][:]
        for j in range(self.n_dim):
            # time update
            self.xhatminus[1][j] = self.xhat[0][j]
            self.Pminus[1][j] = self.P[0][j]+self.Q[j]

            # measurement update
            self.K[1][j] = self.Pminus[1][j]/(self.Pminus[1][j]+self.R[j])
            self.xhat[1][j] = self.xhatminus[1][j]+self.K[1][j]*(measurements[j]-self.xhatminus[1][j])
            self.P[1][j] = (1-self.K[1][j])*self.Pminus[1][j]

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

        # Initialization of the parameters necessary to use the Kalman filter, in
        # the three dimensions of the space.

        n_dim = 3                   # we study the variations in X, Y and Z
        variance = 1e-5             # process variance
        estimate_variance = 1e-4    # estimate of measurement variance. The bigger it is,
                                    # the more accurate the measurements are considered to be.
                                    # The only parameter on which we can act.
        self.kf = KalmanFilter(n_dim, variance, estimate_variance)


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

                # Kalman filter
                measurements = np.concatenate(([float(data_iidre.data[1])],
                                               [float(data_iidre.data[2])],
                                               [float(height_quaternion)]), axis = 0)
                # Flow of the elements measured by the different sensors in order to update
                # the predicted data after their calculation, in the three dimensions of the
                # space.
                self.kf.kalman_estimation(measurements)

                # Writing in file

                self.log_file.write("Time-IIDRE: "+str(data_iidre.data[0])+"\n")
                self.log_file.write("Time-MTi-30: "+str(data_mti.header)+"\n")
                self.log_file.write("Time-LiDAR: "+str(data_lidar.data[0])+"\n")
                self.log_file.write("DATA_IIDRE:"+ str(data_iidre.data[1])+","+
                                                   str(data_iidre.data[2])+"\n")
                self.log_file.write("DATA_MTi-30 - quaternion:"+str(data_mti.quaternion[0])+","+
                                                                str(data_mti.quaternion[1])+","+
                                                                str(data_mti.quaternion[2])+","+
                                                                str(data_mti.quaternion[3])+"\n")
                self.log_file.write("DATA_LiDAR:"+ str(data_lidar.data[1])+"\n")
                self.log_file.write("Coordonnées estimées - Kalman:"+str(self.kf.xhat[1,0])+","+
                                                                     str(self.kf.xhat[1,1])+","+
                                                                     str(self.kf.xhat[1,2])+"\n"+"\n")
        else:
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

        data_iidre.data = [Time, fb_data[1], fb_data[2]]

    def parsing_lidar(self, data_lidar):
        '''
        Splits data at each ',' and takes the second part of data
        which corresponds to the distance measured by LiDAR
        '''
        fb = data_lidar.data.split(",")
        data_lidar.data = fb[1]
        data_lidar.data = [fb[0], int(float(data_lidar.data))]

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
