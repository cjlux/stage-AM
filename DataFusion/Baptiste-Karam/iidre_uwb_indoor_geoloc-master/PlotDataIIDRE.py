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
parser.add_argument('--DIST', action="store_true",
                     help="wether to plot +DIST data or not")
parser.add_argument('--DIST_DBG', action="store_true",
                     help="wether to plot +DIST_DBG data or not")
parser.add_argument('--time', action="store_true",
                     help="wether to plot the histogram of the sampling time intervall")

args = parser.parse_args()
TRAJ = args.TRAJ
DIST = args.DIST
DIST_DBG = args.DIST_DBG
stat = args.stat
time = args.time
data_file = args.data_file

if time:
    for option, name in zip((TRAJ, DIST, DIST_DBG), ('TRAJ', 'DIST', 'DIST_DBG')):
        print(f"Cannot use --{name} whith --time: deactivating --{name}")
        option = False


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

# intialise the distcionnaries with the IDs avec the 4 anchors of the IIDRE system:
#distance = {"556509AF":[], "156509A9":[], "1565010E":[], "556417A1":[]}
#distance_dbg = {"556509AF":[], "156509A9":[], "1565010E":[], "556417A1":[]}

distance = {}
distance_dbg = {}

with open(data_file, "r", encoding="utf8") as F:
    for line in F:
        line = line.strip()     # clean line with \r,\n... at begin or end
        if line.startswith("#"):
            continue   # skip comment lines
        if line.startswith("+MPOS"):
            fb = line.split(";")
            t = fb[0].split(":")[1]
            tag, xyz = fb[1].split(":")
            x, y = map(float, xyz.split(',')[1:])
            data.append([t,x,y])
        elif line.startswith("+DIST:"):
            # +DIST:376660,556509A9,232,0,346,170,-95274,13,
            fb = line.split(";")
            t = fb[0].split(":")[1]
            tag, dist = fb[1].split(":")
            anchor_id = dist.split(',')[1]
            d = float(dist.split(',')[2])
            if anchor_id not in distance: distance[anchor_id] = []
            distance[anchor_id].append([t,d])
        elif  line.startswith("+DIST_DBG"):
            # +DIST:376660,556509A9,232,0,346,170,-95274,13,
            fb = line.split(";")
            t = fb[0].split(":")[1]
            tag, dist = fb[1].split(":")
            anchor_id = dist.split(',')[1]
            d = float(dist.split(',')[2])
            if anchor_id not in distance_dbg: distance_dbg[anchor_id] = []
            distance_dbg[anchor_id].append([t,d])

# Transform the data list into np.ndarray:
data = np.array(data)
print(data)
for (key, val) in distance.items():
   distance[key]=np.array(val)

for (key, val) in distance_dbg.items():
   distance_dbg[key]=np.array(val)

nb_graph = 1 + DIST + DIST_DBG + TRAJ + time

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
axe.set_xlabel("Time [sec]")
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


if DIST:
    image_file += '--DIST'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_title("Distance anchor-tag versus time (Frame +DIST)")
    axe.set_xlabel("Time [s]")
    axe.set_ylabel("distance [cm]")
    colors = ["r", "g", "b", "m"]
    list_keys = list(distance.keys())
    list_keys.sort()
    labels = deepcopy(list_keys)
    plots = []
    if stat:
        # Compute mean and standard deviation:
        averages, std = [], []
        for val in distance.values():
            averages.append(val[:,1].mean(axis=0))
            std.append(val[:,1].std(axis=0))
        # build the stat messages:
        for i in range(len(labels)):
            labels[i] += fr" (mean: {averages[i]:.2f} cm, $\sigma$: {std[i]:.2f} cm)"
    for i, key in enumerate(list_keys):
        val = distance[key]
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        plots.append(axe.plot(T, D, '.-', markersize=MS, linewidth=LW, color=colors[i], label=labels[i]))
    axe.set_ylim(0, ymax)
    axe.grid(True)
    leg = axe.legend(loc='lower right', fontsize=10, framealpha=0.7, ncol=2)
    for text, plot in zip(leg.texts, plots):
        text.set_color(plot[0].get_color())

if DIST_DBG:
    image_file += '--DIST_DBG'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_title("Distance anchor-tag versus time(Frame +DIST_DBG)")
    axe.set_xlabel("Time [s]")
    axe.set_ylabel("distance [cm]")
    colors = ["r", "g", "b", "m"]
    list_keys = list(distance_dbg.keys())
    list_keys.sort()
    labels = deepcopy(list_keys)
    plots = []
    if stat:
        # Compute average and standard deviation:
        averages, std = [], []
        for val in distance_dbg.values():
            averages.append(val[:,1].mean(axis=0))
            std.append(val[:,1].std(axis=0))
        # build the stat messages:
        for i in range(4):
            labels[i] += fr" (mean: {averages[i]:.2f} cm, $\sigma$: {std[i]:.2f} cm)"

    for i, key in enumerate(list_keys):
        val = distance_dbg[key]
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        plots.append(axe.plot(T, D, '.-', markersize=MS, linewidth=LW, color=colors[i], label=labels[i]))
    axe.set_ylim(0, ymax)
    axe.grid(True)
    leg = axe.legend(loc='lower right', fontsize=10,  framealpha=0.7, ncol=2)
    for text, plot in zip(leg.texts, plots):
        text.set_color(plot[0].get_color())


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
plt.savefig(image_file+'.png', dpi=160)
plt.show()
