#
# JLC 2022-06-19 Initial revision
#
# JLC 2022-08-18: Add options --TRAJ, --DIST and --DIST_DBG
#

import time, os, sys
import numpy as np
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy

parser = argparse.ArgumentParser()
parser.add_argument("--data_file", type=str, default=None,
                    help="path of the data file")
parser.add_argument('--stat', action="store_true",
                         help="wether to compute and plot mean and std")
parser.add_argument('--TRAJ', action="store_true",
                     help="wether to plot the trajectory or not")
parser.add_argument('--time', action="store_true",
                     help="wether to plot the histogram of the sampling time intervall")

args = parser.parse_args()
TRAJ = args.TRAJ
stat = args.stat
data_file = args.data_file
time = args.time

if time:
    if (TRAJ):
        print(f"Cannot use --{'TRAJ'} whith --time: deactivating --{'TRAJ'}")
        TRAJ = False
    # for option, name in zip((TRAJ), ('TRAJ')):
    #     print(f"Cannot use --{name} whith --time: deactivating --{name}")
    #     option = False

if data_file is None:
    # list *.txt files so the user can type in the number of the file to process:
    while True:
        data_dir = input("Enter the path of the directory to scan for Data files [or Q for quit]... ")
        if data_dir.upper() == "Q": sys.exit()

        if not os.path.isdir(data_dir):
            print(f"Sorry <{data_dir}> in not a valid path, please retry...")
        else:
            print(f"*.txt files found in the directory <{data_dir}>:")
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

print(f"Opening file <{data_file}>...")

data = []

with open(data_file, "r", encoding="utf8") as F:
    for line in F:
        line = line.strip()     # clean line with \r,\n... at begin or end
        if line.startswith("#"):
            continue   # skip comment lines
        fb  = line.split(';')
        tag, xyz = fb[1].split(":")
        if tag == "+MPOS":
            t = fb[0].split(":")[1]
            x, y = map(float, xyz.split(',')[1:])
            data.append([t,x,y])

# Transform the data list into np.ndarray:
data = np.array(data)
print(data)

nb_graph = 1 + TRAJ + time

sizes   = ((),
           (10,4),
           (10,7),
           (10,9),
           (11,11))
adjusts = ((), #(left,bottom,right,top,wspace,hspace)
           (0.07,0.16,0.95,0.82,0.2,0.50),
           (0.07,0.09,0.95,0.87,0.2,0.42),
           (0.07,0.06,0.95,0.89,0.2,0.46),
           (0.07,0.06,0.95,0.92,0.2,0.48))

fig, axes = plt.subplots(nb_graph, 1)
plt.subplots_adjust(*adjusts[nb_graph])
fig.set_size_inches(sizes[nb_graph])
fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)
image_file = data_file.replace('.txt','--POS')

ymax = 400
MS = 1.5  # MarkerSize
LW = 0.3  # LineWidth

if nb_graph == 1: axes = [axes]

########################################
# Always plot the positions X & Y:
########################################
num_axe = 0
axe = axes[num_axe]


T, X, Y = data[:,0], data[:,1], data[:,2]
T = (T - T[0])*1e-3

axe.set_title("X & Y pos. versus time (Frame +MPOS)")
axe.set_xlabel("Time [s]")
axe.set_ylabel("Position [cm]")
label_x, label_y = "X pos", "Y pos"
if stat:
    # Compute mean and standard deviation for t,x an y:
    averages, std = [], []
    nbVar = 3
    for var in range(nbVar-1) :
        averages.append(data.mean(axis=0)[var+1])
        std.append(data.std(axis=0)[var+1])
    label_x += fr" (mean: {averages[0]:.2f} cm, $\sigma$: {std[0]:.2f} cm)"
    label_y += fr" (mean: {averages[1]:.2f} cm, $\sigma$: {std[1]:.2f} cm)"

p1 = axe.plot(T, X, '.b-', markersize=MS, linewidth=LW, label=label_x)
p2 = axe.plot(T, Y, '.r-', markersize=MS, linewidth=LW, label=label_y)
axe.set_ylim(0, ymax)
axe.grid(True)
leg = axe.legend(loc='lower right', fontsize=10, framealpha=0.7)
leg.texts[0].set_color(p1[0].get_color())
leg.texts[1].set_color(p2[0].get_color())

if time:
    image_file += '--time'
    num_axe += 1
    axe = axes[num_axe]

    T = (data[1:,0] - data[:-1,0]).astype(int)
    axe.set_title("histogram of acquisition time intervals")

    S = list(set(T))
    S.sort()
    T = T.tolist()
    H = []
    for s in S:
        c = T.count(s)
        H.append(c)
        print(f"{s:4d} ms apparaît {c:3d} fois")
    X = list(range(len(H)))
    #axe.grid(zorder=0)
    axe.bar(X, H)
    axe.set_xlim(-1, max(10, max(X))+1)
    axe.set_xticks(X)
    axe.set_xlabel('acquisition time interval [ms]')
    axe.set_xticklabels(S)
    ymin, ymax = axe.get_ylim()
    for x, y in zip(X, H):
        axe.text(x, y+0.01*ymax, y, ha='center', color='b', size=8)

    print("S", S)
    print("H", H)

if TRAJ:
    image_file += '--TRAJ'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_aspect('equal')
    axe.set_ylim(100, 250)
    axe.set_xlim(100, 250)
    axe.plot(X, Y, '.-m', markersize=MS, linewidth=LW)
    axe.set_title("Trajectory")
    axe.set_xlabel("X posistion [cm]")
    axe.set_ylabel("Y Position [cm]")
    #axe.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:6.1f}"))
    axe.grid(True)

#
plt.savefig(image_file+'.png')
plt.show()
