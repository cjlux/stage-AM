#
# Baptiste & Karam - Interniship 2022-07-06 Initial revision ( based on the code of JLC - PlotData.py for IIDRE)
#
import time, os, sys
import numpy as np
import argparse
import matplotlib.pyplot as plt

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
            alt = line.strip()     # clean line with \r,\n... at begin or end
            if alt.startswith("#"):
                continue   # skip comment lines
            z = float(alt)
            data.append([z])

    data = np.array(data)

    Z = data[:,0]
    Z = Z*1e-1

    fig = plt.figure()
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((11,9))
    fig.suptitle(f"Plot data from file <{data_file}>", fontsize=16)
    axe = fig.add_subplot(111)


    axe.set_title("X & Y pos. versus time")
    axe.plot(np.transpose(np.where(Z)), Z, markersize=0.2, linewidth=1.5, color='b', label="Z pos", linestyle=':')
    axe.set_xlabel("Time")
    axe.set_ylabel("Z Position [mm]")
    axe.set_ylim(0, 150)
    axe.grid(True)
    axe.legend()

    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
