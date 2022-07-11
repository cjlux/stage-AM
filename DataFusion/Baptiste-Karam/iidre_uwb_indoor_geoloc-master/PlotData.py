#
# JLC 2022-06-19 Initial revision
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
    distance = {"556509AF":[], "156509A9":[], "1565010E":[], "556417A1":[]}
    distance_dbg = {"556509AF":[], "156509A9":[], "1565010E":[], "556417A1":[]}

    with open(data_file, "r", encoding="utf8") as F:
        for line in F:
            line = line.strip()     # clean line with \r,\n... at begin or end
            if line.startswith("#"):
                continue   # skip comment lines
            if line.startswith("+MPOS"):
	            tag, xyz = line.split(":")
        	    t, x, y = map(float, xyz.split(',')[:3])
        	    data.append([t,x,y])
            elif  line.startswith("+DIST:"):
                    # +DIST:376660,556509A9,232,0,346,170,-95274,13,
	            tag, dist = line.split(":")
	            anchor_id = dist.split(',')[1]
	            t, d = map(float, (dist.split(',')[0], dist.split(',')[2]))
	            distance[anchor_id].append([t,d])
            elif  line.startswith("+DIST_DBG"):
                    # +DIST:376660,556509A9,232,0,346,170,-95274,13,
	            tag, dist = line.split(":")
	            anchor_id = dist.split(',')[1]
	            t, d = map(float, (dist.split(',')[0], dist.split(',')[2]))
	            distance_dbg[anchor_id].append([t,d])

    data = np.array(data)
    for (key, val) in distance.items():
       distance[key]=np.array(val)
    for (key, val) in distance_dbg.items():
       distance_dbg[key]=np.array(val)

    # Calcul de la moyenne et de la variance pour la position
    moyennes = []                                 # Initialise le tableau des moyennes
    std = []                                      # Initialise le tableau des écart-types
    nbVar = 3
    for var in range(nbVar-1) :                   # Calcule la moyenne et l'écart-type selon les différentes dimensions
        moyennes.append(data.mean(axis=0)[var+1])
        std.append(data.std(axis=0)[var+1])

    T, X, Y = data[:,0], data[:,1], data[:,2]
    T = (T - T[0])*1e-3

    fig, axes = plt.subplots(3,1)
    plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
    fig.set_size_inches((11,9))
    fig.suptitle(f"Plot of {tag} data from file <{data_file}>", fontsize=16)

    ymax = 400
    box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}

    axe = axes[0]
    axe.set_title("X & Y pos. versus time (trame : +MPOS)")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("Position [cm]")
    axe.plot(T, X, markersize=0.2, linewidth=1.5, color='b', label="X pos", linestyle=':')
    axe.plot(T, Y, markersize=0.2, linewidth=1.5, color='r', label="Y pos", linestyle=':')
    axe.set_ylim(0, ymax)
    axe.grid(True)
    axe.legend()
    axe.text(0, ymax * 0.98,
            fr"$\bar x$: {moyennes[0]:.2f} cm, $\sigma$: {std[0]:.2f} cm" "\n"
            fr"$\bar y$: {moyennes[1]:.2f} cm, $\sigma$: {std[1]:.2f} cm",
            verticalalignment ='top', horizontalalignment ='left', fontsize=9, bbox=box)

    # Calcul de la moyenne et de la variance pour les distances
    moyennes = []                                       # Initialise le tableau des moyennes
    std = []                                            # Initialise le tableau des écart-types
    for (key, val) in enumerate(distance.items()):      # Calcule la moyenne et l'écart-type selon les différentes dimensions
        moyennes.append(distance[val[0]][:,1].mean(axis=0))
        std.append(distance[val[0]][:,1].std(axis=0))

    axe = axes[1]
    axe.set_title("Distance anchor-tag (trame : +DIST)")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("distance [cm]")
    colors = ["r", "g", "b", "m"]
    for i, (key, val) in enumerate(distance.items()):
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        axe.plot(T, D, markersize=0.2, linewidth=1.5, color=colors[i], label=key, linestyle=':')
    axe.set_ylim(0, ymax)
    axe.grid(True)
    axe.legend()
    axe.text(0, ymax * 0.98,
            fr"$\bar x_ 1$: {moyennes[0]:.2f} cm, $\sigma_ 1$: {std[0]:.2f} cm" "\n"
            fr"$\bar x_ 2$: {moyennes[1]:.2f} cm, $\sigma_ 2$: {std[1]:.2f} cm",
            verticalalignment ='top', horizontalalignment ='left', fontsize=9, bbox=box)
    axe.text(T[-1] * 0.5, ymax * 0.98,
            fr"$\bar x_ 3$: {moyennes[2]:.2f} cm, $\sigma_ 3$: {std[2]:.2f} cm" "\n"
            fr"$\bar x_ 4$: {moyennes[3]:.2f} cm, $\sigma_ 4$: {std[3]:.2f} cm",
            verticalalignment ='top', horizontalalignment ='left', fontsize=9, bbox=box)

    '''
    axe = axes[2]
    axe.set_title("Distance anchor-tag (trame : +DIST_DBG)")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("distance [cm]")
    colors = ["r", "g", "b", "m"]
    for i, (key, val) in enumerate(distance_dbg.items()):
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        axe.plot(T, D, markersize=0.2, linewidth=1.5, color=colors[i], label=key, linestyle=':')
    axe.set_ylim(0, 350)
    axe.grid(True)
    axe.legend()


    axe = axes[2]
    axe.set_aspect('equal')
    axe.set_ylim(0, 300)
    axe.set_xlim(0, 300)
    axe.plot(X, Y, markersize=0.2, linewidth=1.5, color='m')
    axe.set_title("Trajectory")
    axe.set_xlabel("X posistion [cm]")
    axe.set_ylabel("Y Position [cm]")
    #axe.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:6.1f}"))
    axe.grid(True)
    '''


    #
    plt.savefig(data_file.replace('.txt','.png'))
    plt.show()
