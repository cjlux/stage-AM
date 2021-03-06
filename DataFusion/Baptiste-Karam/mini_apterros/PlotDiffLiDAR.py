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

    data_brut = []
    data_traite = []
    time = []

    with open(data_file, "r", encoding="utf8") as F:
        for line in F:
            alt = line.strip()     # clean line with \r,\n... at begin or end

            fb = alt.split(":")
            fb_cmd = fb[0]

            if fb_cmd == "DATA_LiDAR ":
                data_brut.append(int(fb[1].replace(" ", "")))
            elif fb_cmd == "Nouvelles coordonn??es ":
                height = fb[1].split("[")
                real_height = height[6]
                data_traite.append(float(real_height.replace("]", "")))
            elif fb_cmd == "Time":
                time.append(float(fb[1].replace(" ", "")))

            if alt.startswith("#"):
                continue   # skip comment lines

    data_brut = np.array(data_brut)
    data_traite = np.array(data_traite)
    data_moy = (data_brut+data_traite)/2
    time = np.array(time)

    dt_array = time[1:]-time[:-1]
    dt, dt_mean, dt_std = dt_array[0]*1e3, dt_array.mean()*1e3, dt_array.std()*1e3

    if plot_by_rank:
        time = range(len(data_brut))
        x_label = "rank"
    else:
        time -= time[0]
        x_label = "time [second]"

    # fig, axes = plt.subplots(2,1)
    # plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    # fig.set_size_inches((8,6))
    # fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
    # marker_size = 5 if len(data_brut) <= 30 else 1

    # axe = axes[0]
    # axe.set_title("Z (ground distance in the direction of the LiDAR sight)")
    # axe.plot(time, data_brut, '.:b', markersize=marker_size, linewidth=0.3, label="Z")
    # axe.set_ylabel("distance [mm]")
    # axe.set_xlabel(x_label)
    # ymax = 1300
    # axe.set_ylim(0, ymax)
    # if stat:
    #     z_mean, z_std = data_brut.mean(), data_brut.std()
    #     z_min, z_max = data_brut.min(), data_brut.max()
    #     text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
    #     text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
    #     text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
    #     print(text1)
    #     print(text2)
    #     box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
    #     axe.text(0, ymax*.98,
    #              fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
    #              fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
    #              va='top', ha ='left', fontsize=9, bbox=box)
    # axe.grid(True)
    # axe.legend()

    # axe = axes[1]
    # axe.set_title("Z (ground distance in the direction of the LiDAR sight corrected by the MTi's data)")
    # axe.plot(time, data_traite, '.:b', markersize=marker_size, linewidth=0.3, label="Z")
    # axe.set_ylabel("distance [mm]")
    # axe.set_xlabel(x_label)
    # ymax = 1300
    # axe.set_ylim(0, ymax)
    # if stat:
    #     z_mean, z_std = data_traite.mean(), data_traite.std()
    #     z_min, z_max = data_traite.min(), data_traite.max()
    #     text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
    #     text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
    #     text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
    #     print(text1)
    #     print(text2)
    #     box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
    #     axe.text(0, ymax*.98,
    #              fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
    #              fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
    #              va='top', ha ='left', fontsize=9, bbox=box)
    # axe.grid(True)
    # axe.legend()
    #

    fig = plt.figure()
    #plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((8,6))
    fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
    axe = plt.subplot(111)

    marker_size = 5 if len(data_brut) <= 30 else 1
    axe.set_title("Z (ground distance in the direction of the LiDAR sight and corrected by the MTi's data)")
    axe.plot(time, data_brut, '.:b', markersize=marker_size, linewidth=0.3, label="Z LiDAR")
    axe.plot(time, data_traite, '.:r', markersize=marker_size, linewidth=0.3, label="Z real")
    axe.plot(time, data_moy, '.:g', markersize=marker_size, linewidth=0.3, label="Z mean")
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    ymax = 1300
    axe.set_ylim(0, ymax)
    if stat:
        z_mean, z_std = data_brut.mean(), data_brut.std()
        z_min, z_max = data_brut.min(), data_brut.max()
        text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
        text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box_original = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax*.98,
                 fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box_original)

        z_mean, z_std = data_traite.mean(), data_traite.std()
        z_min, z_max = data_traite.min(), data_traite.max()
        text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
        text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box_new = {'facecolor': (.9,.8,.8,.5) , 'edgecolor':'red', 'boxstyle': 'square'}
        axe.text(0, ymax*.85,
                 fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box_new)

        z_mean, z_std = data_moy.mean(), data_moy.std()
        z_min, z_max = data_moy.min(), data_moy.max()
        text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
        text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
        text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
        print(text1)
        print(text2)
        box_mean = {'facecolor': (.8,.9,.8,.5) , 'edgecolor':'green', 'boxstyle': 'square'}
        axe.text(0, ymax*.72,
                 fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
                 va='top', ha ='left', fontsize=9, bbox=box_mean)
    axe.grid(True)
    axe.legend()

    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
