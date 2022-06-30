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
            if line.startswith("#"):
                continue   # skip comment lines
            x_acc, y_acc, z_acc = map(float, acc.split(',')[:3])
            data.append([x_acc,y_acc,z_acc])

    # Position définie de manière brute premièrement en attendant d'avoir des valeurs réalistes
    position = np.ones((len(data),3))
    for i in range(len(data)) :
        position[i] = position[i]*i

    data = np.array(data)

    import matplotlib.pyplot as plt
    from matplotlib import ticker

    X_acc, Y_acc, Z_acc = data[:,0], data[:,1], data[:,2]
    X_pos, Y_pos, Z_pos = position[:,0], position[:,1], position[:,2]

    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((11,9))
    fig.suptitle(f"Plot data from file <{data_file}>", fontsize=16)
    axe = fig.add_subplot(111, projection='3d')

    axe.set_ylim(0, 300)
    axe.set_xlim(0, 300)
    axe.set_zlim(0, 300)
    axe.quiver(X_pos, Y_pos, Z_pos, X_acc, Y_acc, Z_acc,color='g')
    axe.plot(X_pos, Y_pos, Z_pos, markersize=0.2, linestyle=':',linewidth=1, color='m')
    axe.set_title("Acceleration")
    axe.set_xlabel("X posistion [cm]")
    axe.set_ylabel("Y Position [cm]")
    axe.set_zlabel("Z Position [cm]")
    axe.grid(True)


    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
