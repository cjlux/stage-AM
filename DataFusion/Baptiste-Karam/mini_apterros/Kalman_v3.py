# source : https://pykalman.github.io/

import matplotlib.pyplot as plt
import numpy as np
from pykalman import KalmanFilter
import argparse, os, time

def kalman_function():
    '''
    Reading of the log_file (the .txt file where the data are stored) and the
    data related to the position of the module over time in order to process
    them using a Kalman filter.
    '''
    # Get the time of program's execution
    start_time = time.time()

    measurements = []
    Time = []

    with open(data_file, "r", encoding="utf8") as file:
        for line in file:
            alt = line.strip()     # clean line with \r,\n... at begin or end

            fb = alt.split(":")
            fb_cmd = fb[0]

            if fb_cmd == "Nouvelles coordonnées - quaternion":
                x, y, z = map(float, fb[1].split(',')[:])
                measurements.append([x,y,z])
            elif fb_cmd == "Time-MTi-30":
                Time.append(float(fb[1].replace(" ", "")))

            if alt.startswith("#"):
                continue           # skip comment lines

    measurements = np.array(measurements)
    X, Y, Z = measurements[:,0]*10, measurements[:,1]*10, measurements[:,2]

    Time = np.array(Time)
    dt_array = Time[1:]-Time[:-1]
    dt, dt_mean, dt_std = dt_array[0]*1e3, dt_array.mean()*1e3, dt_array.std()*1e3

    if plot_by_rank:
        Time = range(len(measurements))
        x_label = "rank"
    else:
        Time -= Time[0]
        x_label = "Time [s]"

    delta_t = 1.0/60
    kf_x = KalmanFilter(transition_matrices = [[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]],
                        observation_matrices = [1, 0, 0])
    kf_x = kf_x.em(X, n_iter=1)
    (predictions_x, filtered_state_covariances_x) = kf_x.filter(X)

    kf_y = KalmanFilter(transition_matrices = [[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]],
                        observation_matrices = [1, 0, 0])
    kf_y = kf_y.em(Y, n_iter=1)
    (predictions_y, filtered_state_covariances_y) = kf_y.filter(Y)

    kf_z = KalmanFilter(transition_matrices = [[1, delta_t, 0], [0, 1, delta_t], [0, 0, 1]],
                        observation_matrices = [1, 0, 0])
    kf_z = kf_z.em(Z, n_iter=1)
    (predictions_z, filtered_state_covariances_z) = kf_z.filter(Z)

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
    axe.plot(Time, X, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(Time, predictions_x[:,0], '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
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

        x_mean_prediction, x_std_prediction = predictions_x[:,0].mean(), predictions_x[:,0].std()
        x_min_prediction, x_max_prediction = predictions_x[:,0].min(), predictions_x[:,0].max()
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
    axe.plot(Time, Y, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(Time, predictions_y[:,0], '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
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

        y_mean_prediction, y_std_prediction = predictions_y[:,0].mean(), predictions_y[:,0].std()
        y_min_prediction, y_max_prediction = predictions_y[:,0].min(), predictions_y[:,0].max()
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
    axe.plot(Time, Z, '.:b', markersize=marker_size, linewidth=0.3, label="Measurements")
    axe.plot(Time, predictions_z[:,0], '.-r', markersize=marker_size, linewidth=0.3, label = 'Kalman Filter Prediction')
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

        z_mean_prediction, z_std_prediction = predictions_z[:,0].mean(), predictions_z[:,0].std()
        z_min_prediction, z_max_prediction = predictions_z[:,0].min(), predictions_z[:,0].max()
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
    print("--- %s seconds ---" % (time.time() - start_time))
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()

if __name__ == '__main__':

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
