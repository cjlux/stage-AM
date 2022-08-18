#
# JLC 2022-06-19 Initial revision
#
# JLC 2022-08-18: Add options --TRAJ, --DIST and --DIST_DBG
#

import time, os, sys
import numpy as np
import argparse
import matplotlib.pyplot as plt

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

args = parser.parse_args()
TRAJ = args.TRAJ
DIST = args.DIST
DIST_DBG = args.DIST_DBG
stat = args.stat
data_file = args.data_file


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

# Transform the data list into np.ndarray:
data = np.array(data)

for (key, val) in distance.items():
   distance[key]=np.array(val)

for (key, val) in distance_dbg.items():
   distance_dbg[key]=np.array(val)

# Compute mean and standard deviation:
moyennes = []                         
std = []                              
nbVar = 3
for var in range(nbVar-1) :                   
    moyennes.append(data.mean(axis=0)[var+1])
    std.append(data.std(axis=0)[var+1])


T, X, Y = data[:,0], data[:,1], data[:,2]
T = (T - T[0])*1e-3

nb_graph = 1 + DIST + DIST_DBG + TRAJ
sizes   = ((),
           (10,4),
           (10,7),
           (10,9),
           (11,11))
adjusts = ((), #(left,bottom,right,top,wspace,hspace)
           (0.07,0.16,0.95,0.82,0.2,0.50),
           (0.07,0.09,0.95,0.87,0.2,0.42),
           (0.07,0.06,0.95,0.89,0.2,0.46)) 

fig, axes = plt.subplots(nb_graph, 1)
plt.subplots_adjust(*adjusts[nb_graph])
fig.set_size_inches(sizes[nb_graph])
fig.suptitle(f"Data from <{os.path.basename(data_file)}>", fontsize=14)

ymax = 400
image_file = data_file.replace('.txt','--POS')

if nb_graph == 1: axes = [axes]

num_axe = 0
axe = axes[num_axe]
axe.set_title("X & Y pos. versus time (Frame +MPOS)")
axe.set_xlabel("Time [sec]")
axe.set_ylabel("Position [cm]")
label_x, label_y = "X pos", "Y pos"
if stat:
    label_x += fr" (mean: {moyennes[0]:.2f} cm, $\sigma$: {std[0]:.2f} cm)"
    label_y += fr" (mean: {moyennes[1]:.2f} cm, $\sigma$: {std[1]:.2f} cm)"
p1 = axe.plot(T, X, markersize=0.2, linewidth=1.5, color='b', label=label_x, linestyle=':')
p2 = axe.plot(T, Y, markersize=0.2, linewidth=1.5, color='r', label=label_y, linestyle=':')
axe.set_ylim(0, ymax)
axe.grid(True)
leg = axe.legend(loc='lower right', fontsize=8, framealpha=0.7)
leg.texts[0].set_color(p1[0].get_color())
leg.texts[1].set_color(p2[0].get_color())

if DIST:
    image_file += '--DIST'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_title("Distance anchor-tag (Frame +DIST)")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("distance [cm]")
    colors = ["r", "g", "b", "m"]
    labels = list(distance.keys())
    plots = []
    if stat:
        # Compute mean and standard deviation:
        moyennes = []                                       
        std = []                                            
        for (key, val) in enumerate(distance.items()):      
            moyennes.append(distance[val[0]][:,1].mean(axis=0))
            std.append(distance[val[0]][:,1].std(axis=0))
        # build the stat messages:
        for i in range(4):
            labels[i] += fr" (mean: {moyennes[i]:.2f} cm, $\sigma$: {std[i]:.2f} cm)"
    for i, (key, val) in enumerate(distance.items()):
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        plots.append(axe.plot(T, D, markersize=0.2, linewidth=1.5, color=colors[i], label=labels[i], linestyle=':'))
    axe.set_ylim(0, ymax)
    axe.grid(True)
    leg = axe.legend(loc='lower right', fontsize=8, framealpha=0.7, ncol=2)
    for text, plot in zip(leg.texts, plots):
        text.set_color(plot[0].get_color())

if DIST_DBG:
    image_file += '--DIST_DBG'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_title("Distance anchor-tag (Frame +DIST_DBG)")
    axe.set_xlabel("Time [sec]")
    axe.set_ylabel("distance [cm]")
    plots, colors = [], ["r", "g", "b", "m"]
    for i, (key, val) in enumerate(distance_dbg.items()):
        T, D = val[:,0], val[:,1]
        T = (T - T[0])*1e-3
        plots.append(axe.plot(T, D, markersize=0.2, linewidth=1.5, color=colors[i], label=key, linestyle=':'))
    axe.set_ylim(0, 350)
    axe.grid(True)
    axe.legend(loc='lower right', fontsize=8,  framealpha=0.7)
    for text, plot in zip(leg.texts, plots):
        text.set_color(plot[0].get_color())


if TRAJ:
    image_file += '--TRAJ'
    num_axe += 1
    axe = axes[num_axe]
    axe.set_aspect('equal')
    axe.set_ylim(100, 300)
    axe.set_xlim(100, 300)
    axe.plot(X, Y, markersize=0.2, linewidth=1.5, color='m')
    axe.set_title("Trajectory")
    axe.set_xlabel("X posistion [cm]")
    axe.set_ylabel("Y Position [cm]")
    #axe.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:6.1f}"))
    axe.grid(True)
    
#
plt.savefig(image_file+'.png')
plt.show()
