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

    data = []
    time = []

    with open(data_file, "r", encoding="utf8") as F:
        for line in F:
            alt = line.strip()     # clean line with \r,\n... at begin or end

            fb = alt.split(":")
            fb_cmd = fb[0]

            if fb_cmd == "DATA_MTi-30 - euler":
                x, y, z = map(float, fb[1].split(',')[:])
                data.append([x,y,z])
            elif fb_cmd == "Time-MTi-30":
                time.append(float(fb[1].replace(" ", "")))

            if alt.startswith("#"):
                continue   # skip comment lines

    data = np.array(data)
    data = np.degrees(data)
    time = np.array(time)

    X, Y, Z = data[:,0], data[:,1], data[:,2]

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
    marker_size = 5 if len(data) <= 30 else 1

    axe = axes[0]
    axe.set_title("Roll")
    axe.plot(time, X, '.:b', markersize=marker_size, linewidth=0.3, label="Roll euler")
    axe.set_ylabel("Euler angle [degrees]")
    axe.set_xlabel(x_label)
    ymax = 90
    axe.set_ylim(-ymax, ymax)
    if stat:
        x_mean, x_std = X.mean(), X.std()
        x_min, x_max = X.min(), X.max()
        text1 = f"x_mean: {x_mean:.1f} deg, x_std: {x_std:.1f} deg"
        text1 += f"(min, max): ({x_min:.1f}, {x_max:.1f}) deg"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax*.94,
                 fr"mean$_x$: {x_mean:.1f} deg, $\sigma_x$: {x_std:.1f} deg, " +
                 fr"(x$_{{min}}$, $x_{{max}}$): ({x_min:.1f}, {x_max:.1f}) deg"+ "\n" + text2,
                 va='top', ha ='left', fontsize=14, bbox=box)
    axe.grid(True)
    axe.legend()

    axe = axes[1]
    axe.set_title("Pitch")
    axe.plot(time, Y, '.:r', markersize=marker_size, linewidth=0.3, label="Pitch euler")
    axe.set_ylabel("Euler angle [degrees]")
    axe.set_xlabel(x_label)
    ymax = 90
    axe.set_ylim(-ymax, ymax)
    if stat:
        y_mean, y_std = Y.mean(), Y.std()
        y_min, y_max = Y.min(), Y.max()
        text1 = f"x_mean: {y_mean:.1f} deg, y_std: {y_std:.1f} deg"
        text1 += f"(min, max): ({y_min:.1f}, {y_max:.1f}) deg"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'red', 'boxstyle': 'square'}
        axe.text(0, ymax*.94,
                 fr"mean$_y$: {y_mean:.1f} deg, $\sigma_y$: {y_std:.1f} deg, " +
                 fr"(y$_{{min}}$, $y_{{max}}$): ({y_min:.1f}, {y_max:.1f}) deg"+ "\n" + text2,
                 va='top', ha ='left', fontsize=14, bbox=box)
    axe.grid(True)
    axe.legend()

    axe = axes[2]
    axe.set_title("Yaw")
    axe.plot(time, Z, '.:g', markersize=marker_size, linewidth=0.3, label="Yaw euler")
    axe.set_ylabel("Euler angle [degrees]")
    axe.set_xlabel(x_label)
    ymax = 90
    axe.set_ylim(-ymax, ymax)
    if stat:
        z_mean, z_std = Z.mean(), Z.std()
        z_min, z_max = Z.min(), Z.max()
        text1 = f"x_mean: {z_mean:.1f} deg, z_std: {z_std:.1f} deg"
        text1 += f"(min, max): ({z_min:.1f}, {z_max:.1f}) deg"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'green', 'boxstyle': 'square'}
        axe.text(0, ymax*.94,
                 fr"mean$_z$: {z_mean:.1f} deg, $\sigma_z$: {z_std:.1f} deg, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min:.1f}, {z_max:.1f}) deg"+ "\n" + text2,
                 va='top', ha ='left', fontsize=14, bbox=box)
    axe.grid(True)
    axe.legend()

    plt.savefig(data_file.replace('.txt','.png'), dpi=160)
    plt.show()