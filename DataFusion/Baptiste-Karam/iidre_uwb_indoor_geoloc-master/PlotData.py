#
# JLC 2022-06-19 Initial revision
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
            line = line.strip()     # clean line with \r,\n... at begin or end
            if line.startswith("#"):
                continue   # skip comment lines
            tag, xyz = line.split(":")
            t, x, y = map(float, xyz.split(',')[:3])
            data.append([t,x,y])          

    data = np.array(data)
    
    # Plot Acc, Gyr and Mag
    #
    
    import matplotlib.pyplot as plt
    from matplotlib import ticker

    T, X, Y = data[:,0], data[:,1], data[:,2]
    T = (T - T[0])*1e-3
    
    fig, axes = plt.subplots(2,1)
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((11,9))
    fig.suptitle(f"Plot of {tag} data from file <{data_file}>", fontsize=16)

    axe = axes[0]
    axe.set_title("X & Y pos. versus time")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("Position [cm]")
    axe.plot(T, X, markersize=0.2, linewidth=1.5, color='b', label="X pos")
    axe.plot(T, Y, markersize=0.2, linewidth=1.5, color='r', label="Y pos")
    axe.set_ylim(0, 300)
    axe.grid(True)
    axe.legend()

    axe = axes[1]
    axe.set_aspect('equal')
    axe.set_ylim(0, 300)
    axe.set_xlim(0, 300)
    axe.plot(X, Y, markersize=0.2, linewidth=1.5, color='m')
    axe.set_title("Trajectory")
    axe.set_xlabel("X posistion [cm]")
    axe.set_ylabel("Y Position [cm]")
    #axe.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:6.1f}"))
    axe.grid(True)

    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
