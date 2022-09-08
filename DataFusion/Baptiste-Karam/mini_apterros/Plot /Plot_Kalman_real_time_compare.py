#
# Baptiste & Karam - Interniship 2022-07-06 Initial revision ( based on the code of JLC - PlotData.py for IIDRE)
#
import time, os, sys
import numpy as np
import argparse
import matplotlib.pyplot as plt

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

    data_measurement = []
    data_Kalman = []
    time = []

    with open(data_file, "r", encoding="utf8") as F:
        for line in F:
            alt = line.strip()     # clean line with \r,\n... at begin or end

            fb = alt.split(":")
            fb_cmd = fb[0]

            if fb_cmd == "Nouvelles coordonnées - quaternion":
                x, y, z = map(float, fb[1].split(',')[:])
                data_measurement.append([x,y,z])
            elif fb_cmd == "Coordonnées estimées - Kalman":
                x, y, z = map(float, fb[1].split(',')[:])
                data_Kalman.append([x,y,z])
            elif fb_cmd == "Time-MTi-30":
                time.append(float(fb[1].replace(" ", "")))

            if alt.startswith("#"):
                continue   # skip comment lines

    data_measurement = np.array(data_measurement)
    data_Kalman = np.array(data_Kalman)
    time = np.array(time)

    X_measurement, Y_measurement, Z_measurement = data_measurement[:,0], data_measurement[:,1], data_measurement[:,2]
    X_Kalman, Y_Kalman, Z_Kalman = data_Kalman[:,0], data_Kalman[:,1], data_Kalman[:,2]

    dt_array = time[1:]-time[:-1]
    dt, dt_mean, dt_std = dt_array[0]*1e3, dt_array.mean()*1e3, dt_array.std()*1e3

    if plot_by_rank:
        time = range(len(data))
        x_label = "rank"
    else:
        time -= time[0]
        x_label = "Time [s]"

    fig, axes = plt.subplots(3,1)
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((8,6))
    fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
    marker_size = 1.5 if len(data_measurement) <= 30 else 1
    ymin_position = -500
    ymax_position = 1000
    ymax_height = 1500

    axe = axes[0]
    axe.set_title("X (corrected by the MTi's data and estimated with the Kalman filter)")
    axe.plot(time, X_measurement, '.:b', markersize=marker_size, linewidth=0.3, label="X measurement")
    axe.plot(time, X_Kalman, '.:r', markersize=marker_size, linewidth=0.3, label="X estimation")
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(ymin_position, ymax_position)
    if stat:
        x_mean_mes, x_std_mes = X_measurement.mean(), X_measurement.std()
        x_min_mes, x_max_mes = X_measurement.min(), X_measurement.max()
        text1 = f"x_mean_mes: {x_mean_mes:.1f} cm, x_std_mes: {x_std_mes:.1f} cm"
        text1 += f"(min, max): ({x_min_mes:.1f}, {x_max_mes:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax_position*.98,
                 fr"mean$_x$: {x_mean_mes:.1f} cm, $\sigma_x$: {x_std_mes:.1f} cm, " +
                 fr"(x$_{{min}}$, $x_{{max}}$): ({x_min_mes:.1f}, {x_max_mes:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box)

        x_mean_est, x_std_est = X_Kalman.mean(), X_Kalman.std()
        x_min_est, x_max_est = X_Kalman.min(), X_Kalman.max()
        text1 = f"x_mean_est: {x_mean_est*.1:.1f} cm, x_mean_est: {x_mean_est*.1:.1f} cm"
        text1 += f"(min, max): ({x_mean_est*.1:.1f}, {x_mean_est*.1:.1f}) cm"
        print(text1)
        box_new = {'facecolor': (.9,.8,.8,.5) , 'edgecolor':'red', 'boxstyle': 'square'}
        axe.text(0, ymax_position*.65,
                  fr"mean$_x$: {x_mean_est*.1:.1f} cm, $\sigma_x$: {x_mean_est*.1:.1f} cm, " +
                  fr"(x$_{{min}}$, $x_{{max}}$): ({x_mean_est*.1:.1f}, {x_mean_est*.1:.1f}) cm"+ "\n" + text2,
                  va='top', ha ='left', fontsize=9, bbox=box_new)
    axe.grid(True)
    axe.legend()

    axe = axes[1]
    axe.set_title("Y (corrected by the MTi's data and estimated with the Kalman filter)")
    axe.plot(time, Y_measurement, '.:b', markersize=marker_size, linewidth=0.3, label="Y measurement")
    axe.plot(time, Y_Kalman, '.:r', markersize=marker_size, linewidth=0.3, label="Y estimation")
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(ymin_position, ymax_position)
    if stat:
        y_mean_mes, y_std_mes = Y_measurement.mean(), Y_measurement.std()
        y_min_mes, y_max_mes = Y_measurement.min(), Y_measurement.max()
        text1 = f"y_mean_mes: {y_mean_mes:.1f} cm, y_std_mes: {y_std_mes:.1f} cm"
        text1 += f"(min, max): ({y_min_mes:.1f}, {y_max_mes:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax_position*.98,
                 fr"mean$_y$: {y_mean_mes:.1f} cm, $\sigma_y$: {y_std_mes:.1f} cm, " +
                 fr"(y$_{{min}}$, $y_{{max}}$): ({y_min_mes:.1f}, {y_max_mes:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box)

        y_mean_est, y_std_est = Y_Kalman.mean(), Y_Kalman.std()
        y_min_est, y_max_est = Y_Kalman.min(), Y_Kalman.max()
        text1 = f"y_mean_est: {y_mean_est*.1:.1f} cm, y_mean_est: {y_mean_est*.1:.1f} cm"
        text1 += f"(min, max): ({y_mean_est*.1:.1f}, {y_mean_est*.1:.1f}) cm"
        print(text1)
        box_new = {'facecolor': (.9,.8,.8,.5) , 'edgecolor':'red', 'boxstyle': 'square'}
        axe.text(0, ymax_position*.60,
                  fr"mean$_y$: {y_mean_est*.1:.1f} cm, $\sigma_y$: {y_mean_est*.1:.1f} cm, " +
                  fr"(y$_{{min}}$, $y_{{max}}$): ({y_mean_est*.1:.1f}, {y_mean_est*.1:.1f}) cm"+ "\n" + text2,
                  va='top', ha ='left', fontsize=9, bbox=box_new)
    axe.grid(True)
    axe.legend()

    axe = axes[2]
    axe.set_title("Z (corrected by the MTi's data and estimated with the Kalman filter)")
    axe.plot(time, Z_measurement, '.:b', markersize=marker_size, linewidth=0.3, label="Z measurement")
    axe.plot(time, Z_Kalman, '.:r', markersize=marker_size, linewidth=0.3, label="Z estimation")
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    axe.set_ylim(0, ymax_height)
    if stat:
        z_mean_mes, z_std_mes = Z_measurement.mean(), Z_measurement.std()
        z_min_mes, z_max_mes = Z_measurement.min(), Z_measurement.max()
        text1 = f"z_mean_mes: {z_mean_mes:.1f} cm, z_std_mes: {z_std_mes:.1f} cm"
        text1 += f"(min, max): ({z_min_mes:.1f}, {z_max_mes:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax_height*.98,
                 fr"mean$_z$: {z_mean_mes:.1f} cm, $\sigma_z$: {z_std_mes:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min_mes:.1f}, {z_max_mes:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box)

        z_mean_est, z_std_est = Z_Kalman.mean(), Z_Kalman.std()
        z_min_est, z_max_est = Z_Kalman.min(), Z_Kalman.max()
        text1 = f"z_mean_est: {z_mean_est*.1:.1f} cm, z_mean_est: {z_mean_est*.1:.1f} cm"
        text1 += f"(min, max): ({z_mean_est*.1:.1f}, {z_mean_est*.1:.1f}) cm"
        print(text1)
        box_new = {'facecolor': (.9,.8,.8,.5) , 'edgecolor':'red', 'boxstyle': 'square'}
        axe.text(0, ymax_height*.75,
                  fr"mean$_z$: {z_mean_est*.1:.1f} cm, $\sigma_z$: {z_mean_est*.1:.1f} cm, " +
                  fr"(z$_{{min}}$, $z_{{max}}$): ({z_mean_est*.1:.1f}, {z_mean_est*.1:.1f}) cm"+ "\n" + text2,
                  va='top', ha ='left', fontsize=9, bbox=box_new)
    axe.grid(True)
    axe.legend()

    plt.savefig(data_file.replace('.txt','.png'), dpi=160)
    plt.show()
