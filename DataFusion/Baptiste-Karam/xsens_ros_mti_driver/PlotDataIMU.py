#
# Baptiste & Karam - Interniship 2022-06-30 Initial revision ( based on the code of JLC - PlotData.py for IIDRE)
#
import time, os, sys
import numpy as np
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_file", type=str, default="")
    args = parser.parse_args()
    data_file = vars(args)["data_file"]

    if data_file == "":
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
            acc = line.strip()     # clean line with \r,\n... at begin or end
            if acc.startswith("#"):
                continue   # skip comment lines
            x_acc, y_acc, z_acc = map(float, acc.split(',')[:3])
            data.append([x_acc,y_acc,z_acc])

    data = np.array(data)

    import matplotlib.pyplot as plt
    from matplotlib import ticker

    X_acc, Y_acc, Z_acc = data[:,0], data[:,1], data[:,2]

    from mpl_toolkits.mplot3d import Axes3D

    fig, axes = plt.subplots(3,1)
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((11,9))
    fig.suptitle(f"Plot data from file <{data_file}>", fontsize=16)

    axe = axes[0]
    axe.set_ylim(0, 15)
    axe.plot(X_acc, markersize=5, marker='x', linestyle = ' ', color='r', label="X acceleration")
    axe.set_title("Acceleration selon X")
    axe.set_xlabel("Time [s]")
    axe.set_ylabel(r"Acceleration [m/$s^2$]")
    axe.legend()
    axe.grid(True)

    axe = axes[1]
    axe.set_ylim(0, 15)
    axe.plot(Y_acc, markersize=5, marker='x', linestyle = ' ', color='g', label="Y acceleration")
    axe.set_title("Acceleration selon Y")
    axe.set_xlabel("Time [s]")
    axe.set_ylabel(r"Acceleration [m/$s^2$]")
    axe.legend()
    axe.grid(True)

    axe = axes[2]
    axe.set_ylim(0, 15)
    axe.plot(Z_acc, markersize=5, marker='x', linestyle = ' ', color='b', label="Z acceleration")
    axe.set_title("Acceleration selon Z")
    axe.set_xlabel("Time [s]")
    axe.set_ylabel(r"Acceleration [m/$s^2$]")
    axe.legend()
    axe.grid(True)
    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
