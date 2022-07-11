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

    with open(data_file, "r", encoding="utf8") as F:
        for line in F:
            alt = line.strip()     # clean line with \r,\n... at begin or end
            if alt.startswith("#"):
                continue   # skip comment lines
            t, z = map(float, alt.split(','))
            data.append([t, z])

    data = np.array(data)

    Z = data[:,1]

    if plot_by_rank:
        T = range(len(Z))
        x_label = "rank"
    else:
        T = data[:,0] - data[0,0]
        x_label = "time [second]"

    fig = plt.figure()
    #plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((8,6))
    fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
    axe = plt.subplot(111)

    marker_size = 5 if len(Z) <= 30 else 1
    axe.set_title("Z (ground distance in the direction of the LiDAR sight)")
    axe.plot(T, Z, '.:b', markersize=marker_size, linewidth=0.3, label="Z")
    axe.set_ylabel("distance [mm]")
    axe.set_xlabel(x_label)
    ymax = 1300
    axe.set_ylim(0, ymax)
    if stat:
        z_mean, z_std = Z.mean(), Z.std()
        z_min, z_max = Z.min(), Z.max()
        text = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
        text += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
        print(text)
        box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
        axe.text(0, ymax*.98,
                 fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
                 fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm",
                 va='top', ha ='left', fontsize=9, bbox=box)
    axe.grid(True)
    axe.legend()
    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
