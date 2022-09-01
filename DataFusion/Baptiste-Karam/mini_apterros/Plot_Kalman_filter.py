
# source : https://github.com/zziz/kalman-filter
import numpy as np
import random

class KalmanFilter(object):
    '''
    This class provides the first Kalman filter estimates and those recalibrated
    using measurements from the IIDRE, LiDAR and MTi-30 sensors.
    '''
    def __init__(self, F = None, B = None, H = None, Q = None, R = None,
                 P = None, x0 = None, w = None):
        '''
        Nota Bene :
          z0 : observation or measurement of the process

        Parameters :
          n : number of columns of F
          m : number of lines of H
          F : matrix that connects the previous state k-1 to the current state k
          B : matrix that relates the control input u(k) to the state x(k), set
          to 0 by default
          H : matrix that relates the state x(k) to the measure z(k)
          Q : covariance matrix of the process noise, defined by an identity
          matrix of size n by default
          R : covariance matrix of the measurement noise, defined by an identity
          matrix of size m by default
          P : estimation matrix of the covariance of the error, defined by an
          identity matrix of size n by default
          x0 : estimation of the state
          w : process noise
        '''
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[0]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.m) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        self.w = np.zeros((self.m, 1)) if w is None else w

    def predict(self, u = 0):
        '''
        It starts by determining the value of x, which defines the new predicted.

        Note that, in our case, u = 0 since we only consider the position returned
        by the sensors and not the forces applied to the tag.

        Then, the a priori estimation matrix of the covariance of the error P is
        calculated.
        '''
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u) + self.w
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        '''
        The innovation is defined here as a comparison between the predicted
        value and the measured value. The covariance associated with this column
        matrix is also calculated.

        This allows us to determine the optimal Kalman gain and thus the updated
        state.

        Finally an identity matrix of size n is initialized to update the
        covariance matrix P.
        '''
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(I - np.dot(K, self.H), self.P)

def kalman_function():
    '''
    Reading of the log_file (the .txt file where the data are stored) and the
    data related to the position of the module over time in order to process
    them using a Kalman filter.
    '''
    # delta_t must be equal to 1s for the discretization of the navigation equations
    # according to the document : https://cnriut2019.sciencesconf.org/data/6_15H00_FA010.pdf
    delta_t = 1.0/60
    # For the values of the matrix F, these are the values recommended by this
    # pdf https://cnriut2019.sciencesconf.org/data/6_15H00_FA010.pdf
    F_x = np.array([[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]])
    F_y = np.array([[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]])
    F_z = np.array([[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]])
    H_x = np.array([1, 0, 0]).reshape(1, 3)
    H_y = np.array([1, 0, 0]).reshape(1, 3)
    H_z = np.array([1, 0, 0]).reshape(1, 3)

    # For the values of the Q matrix, these are the values recommended by the
    # person in this forum https://robotics.stackexchange.com/questions/11178/kalman-filter-gps-imu
    alpha = 0.1
    Q_x = np.array([[alpha, 0.0, 0.0], [0.0, alpha, 0.0], [0.0, 0.0, alpha]])
    Q_y = np.array([[alpha, 0.0, 0.0], [0.0, alpha, 0.0], [0.0, 0.0, alpha]])
    Q_z = np.array([[alpha, 0.0, 0.0], [0.0, alpha, 0.0], [0.0, 0.0, alpha]])

    # This parameters represent the initial position of the tag
    delta_x = 140
    delta_y = 170
    delta_z = 20
    P_x = np.array([[delta_x, 0.0, 0.0], [0.0, delta_x, 0.0], [0.0, 0.0, delta_x]])
    P_y = np.array([[delta_y, 0.0, 0.0], [0.0, delta_y, 0.0], [0.0, 0.0, delta_y]])
    P_z = np.array([[delta_z, 0.0, 0.0], [0.0, delta_z, 0.0], [0.0, 0.0, delta_z]])

    measurements = []
    time = []

    with open(data_file, "r", encoding="utf8") as file:
        for line in file:
            alt = line.strip()     # clean line with \r,\n... at begin or end

            fb = alt.split(":")
            fb_cmd = fb[0]

            if fb_cmd == "Nouvelles coordonnées - quaternion":
                x, y, z = map(float, fb[1].split(',')[:])
                measurements.append([x,y,z])
            elif fb_cmd == "Time-MTi-30":
                time.append(float(fb[1].replace(" ", "")))

            if alt.startswith("#"):
                continue           # skip comment lines

    measurements = np.array(measurements)
    X, Y, Z = measurements[:,0]*10, measurements[:,1]*10, measurements[:,2]

    time = np.array(time)
    dt_array = time[1:]-time[:-1]
    dt, dt_mean, dt_std = dt_array[0]*1e3, dt_array.mean()*1e3, dt_array.std()*1e3

    if plot_by_rank:
        time = range(len(measurements))
        x_label = "rank"
    else:
        time -= time[0]
        x_label = "Time [s]"

    # Initialization of the parameters necessary to use the Kalman filter, in
    # the three dimensions of the space. Note that the noise of the process w is
    # defined from a Gaussian centered in 0 and of standard deviation equal to
    # what was measured in the case of a stationary tag.
    kf_x = KalmanFilter(F = F_x, H = H_x, Q = Q_x, P = P_x,
                        w = np.array([random.gauss(0, 0.73)]).reshape(1, 1))
    kf_y = KalmanFilter(F = F_y, H = H_y, Q = Q_y, P = P_y,
                        w = np.array([random.gauss(0, 0.53)]).reshape(1, 1))
    kf_z = KalmanFilter(F = F_z, H = H_z, Q = Q_z, P = P_z,
                        w = np.array([random.gauss(0, 0.6)]).reshape(1, 1))
    predictions_x = []
    predictions_y = []
    predictions_z = []

    # Flow of the elements measured by the different sensors in order to update
    # the predicted data after their calculation, in the three dimensions of the
    # space.
    for x in X:
        predictions_x.append(np.dot(H_x,  kf_x.predict())[0])
        kf_x.update(x)
    for y in Y:
        predictions_y.append(np.dot(H_y,  kf_y.predict())[0])
        kf_y.update(y)
    for z in Z:
        predictions_z.append(np.dot(H_z,  kf_z.predict())[0])
        kf_z.update(z)

    # Cast of the elements of the list in array, in the three dimensions of the
    # space.
    predictions_x = np.array(predictions_x)
    predictions_y = np.array(predictions_y)
    predictions_z = np.array(predictions_z)

    fig, axes = plt.subplots(3,1)
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((8,6))
    fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
    marker_size = 1.5 if len(measurements) <= 30 else 1
    ymax_position = 3500
    ymax_height = 1500
    ymin = -500

    axe = axes[0]
    axe.set_title("X's position comparison between measurement and prediction")
    axe.plot(time, X, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(time, predictions_x, '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(ymin, ymax_position)
    if stat:
        x_mean_measurement, x_std_measurement = X.mean(), X.std()
        x_min_measurement, x_max_measurement = X.min(), X.max()
        text1_measurement = f"x_mean_measurement: {x_mean_measurement:.1f} cm, x_std_measurement: {x_std_measurement:.1f} cm"
        text1_measurement += f"(min, max): ({x_min_measurement:.1f}, {x_max_measurement:.1f}) cm"
        text2_measurement = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1_measurement)
        print(text2_measurement)
        box_measurement = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}

        x_mean_prediction, x_std_prediction = predictions_x.mean(), predictions_x.std()
        x_min_prediction, x_max_prediction = predictions_x.min(), predictions_x.max()
        text1_prediction = f"x_mean_prediction: {x_mean_prediction:.1f} cm, x_std_prediction: {x_std_prediction:.1f} cm"
        text1_prediction += f"(min, max): ({x_min_prediction:.1f}, {x_max_prediction:.1f}) cm"
        print(text1_prediction)
        box_prediction = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'red', 'boxstyle': 'square'}

        axe.text(0, ymax_position*.98,
                 fr"mean$_x$: {x_mean_measurement:.1f} cm, $\sigma_x$: {x_std_measurement:.1f} cm, " +
                 fr"(x$_{{min}}$, $x_{{max}}$): ({x_min_measurement:.1f}, {x_max_measurement:.1f}) cm"+ "\n" + text2_measurement,
                 va='top', ha ='left', fontsize=9, bbox=box_measurement)
        axe.text(0, ymax_position*.65,
                 fr"mean$_x$: {x_mean_prediction:.1f} cm, $\sigma_x$: {x_std_prediction:.1f} cm, " +
                 fr"(x$_{{min}}$, $x_{{max}}$): ({x_min_prediction:.1f}, {x_max_prediction:.1f}) cm",
                 va='top', ha ='left', fontsize=9, bbox=box_prediction)
    axe.grid(True)
    axe.legend()

    axe = axes[1]
    axe.set_title("Y's position comparison between measurement and prediction")
    axe.plot(time, Y, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(time, predictions_y, '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(ymin, ymax_position)
    if stat:
        y_mean_measurement, y_std_measurement = Y.mean(), Y.std()
        y_min_measurement, y_max_measurement = Y.min(), Y.max()
        text1_measurement = f"y_mean_measurement: {y_mean_measurement:.1f} cm, y_std_measurement: {y_std_measurement:.1f} cm"
        text1_measurement += f"(min, max): ({y_min_measurement:.1f}, {y_max_measurement:.1f}) cm"
        text2_measurement = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1_measurement)
        print(text2_measurement)
        box_measurement = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}

        y_mean_prediction, y_std_prediction = predictions_y.mean(), predictions_y.std()
        y_min_prediction, y_max_prediction = predictions_y.min(), predictions_y.max()
        text1_prediction = f"y_mean_prediction: {y_mean_prediction:.1f} cm, y_std_prediction: {y_std_prediction:.1f} cm"
        text1_prediction += f"(min, max): ({y_min_prediction:.1f}, {y_max_prediction:.1f}) cm"
        print(text1_prediction)
        box_prediction = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'red', 'boxstyle': 'square'}

        axe.text(0, ymax_position*.98,
                 fr"mean$_y$: {y_mean_measurement:.1f} cm, $\sigma_y$: {y_std_measurement:.1f} cm, " +
                 fr"(y$_{{min}}$, $y_{{max}}$): ({y_min_measurement:.1f}, {y_max_measurement:.1f}) cm"+ "\n" + text2_measurement,
                 va='top', ha ='left', fontsize=9, bbox=box_measurement)
        axe.text(0, ymax_position*.65,
                 fr"mean$_y$: {y_mean_prediction:.1f} cm, $\sigma_y$: {y_std_prediction:.1f} cm, " +
                 fr"(y$_{{min}}$, $y_{{max}}$): ({y_min_prediction:.1f}, {y_max_prediction:.1f}) cm",
                 va='top', ha ='left', fontsize=9, bbox=box_prediction)
    axe.grid(True)
    axe.legend()

    axe = axes[2]
    axe.set_title("Z's position comparison between measurement and prediction")
    axe.plot(time, Z, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(time, predictions_z, '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(0, ymax_height)
    if stat:
        z_mean_measurement, z_std_measurement = Z.mean(), Z.std()
        z_min_measurement, z_max_measurement = Z.min(), Z.max()
        text1_measurement = f"z_mean_measurement: {z_mean_measurement*.1:.1f} cm, z_std_measurement: {z_std_measurement*.1:.1f} cm"
        text1_measurement += f"(min, max): ({z_min_measurement*.1:.1f}, {z_max_measurement*.1:.1f}) cm"
        text2_measurement = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1_measurement)
        print(text2_measurement)
        box_measurement = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}

        z_mean_prediction, z_std_prediction = predictions_z.mean(), predictions_z.std()
        z_min_prediction, z_max_prediction = predictions_z.min(), predictions_z.max()
        text1_prediction = f"z_mean_prediction: {z_mean_prediction*.1:.1f} cm, z_std_prediction: {z_std_prediction*.1:.1f} cm"
        text1_prediction += f"(min, max): ({z_min_prediction*.1:.1f}, {z_max_prediction*.1:.1f}) cm"
        print(text1_prediction)
        box_prediction = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'red', 'boxstyle': 'square'}

        axe.text(0, ymax_height*.98,
                 fr"mean$_z$: {z_mean_measurement*.1:.1f} cm, $\sigma_z$: {z_std_measurement*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min_measurement*.1:.1f}, {z_max_measurement*.1:.1f}) cm"+ "\n" + text2_measurement,
                 va='top', ha ='left', fontsize=9, bbox=box_measurement)
        axe.text(0, ymax_height*.73,
                 fr"mean$_z$: {z_mean_prediction*.1:.1f} cm, $\sigma_z$: {z_std_prediction*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min_prediction*.1:.1f}, {z_max_prediction*.1:.1f}) cm",
                 va='top', ha ='left', fontsize=9, bbox=box_prediction)
    axe.grid(True)
    axe.legend()
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import argparse, os

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_file", type=str, default=None,
                        help="path of the data file")
    parser.add_argument('--stat', action="store_true",
                         help="wether to compute and plot mean and std")
    parser.add_argument('--plot_by_rank', action="store_true",
                         help="drop temporal data and use ranks for abscissas.")
    args = parser.parse_args()
    stat = args.stat
    plot_by_rank = args.plot_by_rank
    data_file = args.data_file

    if data_file is None:
        while True:
            data_dir = input("Enter the path of the directory to scan for Data files [or Q for quit]... ")
            if data_dir.upper() == "Q": sys.exit()

            if not os.path.isdir(data_dir):
                print(f"Sorry <{data_dir}> in not a valid path, please retry...")
            else:
                break
        list_dataFiles = [ f for f in os.listdir(data_dir) if f.endswith('txt')]
        list_dataFiles.sort()
        for i,file in enumerate(list_dataFiles, 1):
            print(f"{i}\t{file}")
        choice = input("Which data file number to process [Q for quit]... ")
        if choice.upper() == "Q":
            sys.exit(0)

        data_file = list_dataFiles[int(choice)-1]
        data_file = os.path.join(data_dir, data_file)

    print(f"Opening log file <{data_file}>...")

    kalman_function()
