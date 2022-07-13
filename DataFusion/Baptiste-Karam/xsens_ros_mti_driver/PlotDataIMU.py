#
# Baptiste & Karam - Interniship 2022-06-30 Initial revision ( based on the code of JLC - PlotData.py for IIDRE)
#
import time, os, sys
import numpy as np
import argparse
import matplotlib.pyplot as plt
from matplotlib import ticker

class Plot_IMU(object):
    '''This class allows to plot the data got by IMU.
    '''

    TYPE_INFO = {'acceleration': ["Acceleration", "X", "Y", "Z", "m/$s^2$"],
                 'angular_velocity': ["Angular Veolicty", "$\theta$", "$\phi$", "$\psi$", "rad/s"],
                 'magnetic': ["Magnetic", "X", "Y", "Z", "rad/$\mu$T"],
                 'euler': ["Euler angle", "X", "Y", "Z", "rad"]}

    def __init__(self, type_info, data_file, stat):
        '''Parameters:
             type_info:str: the IMU informations type, see allowed values in TYPE_INFO.
        '''
        self.type_info = type_info
        self.data_file = data_file
        self.stat = stat

    def compute(self):
        '''
           Request the file you wish to process.
           Extracts the information from this file.
        '''
        if self.data_file == "":
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

            self.data_file = list_dataFiles[int(choice)-1]
            data_file = os.path.join(data_dir, self.data_file)

        print(f"Opening log file <{self.data_file}>...")

        data = []

        with open(self.data_file, "r", encoding="utf8") as F:
            for line in F:
                line = line.strip()     # clean line with \r,\n... at begin or end
                if line.startswith("#"):
                    continue   # skip comment lines
                type_time, time_type, info = line.split(":")
                x, y, z = map(float, info.split(',')[:])
                time = float(time_type.split(',')[0])
                data.append([time,x,y,z])

        data = np.array(data)
        self.disp_data(data)

    def disp_data(self, data):
        '''
           Processes the extracted information.
           The display varies according to the type of information processed and
           the options chosen (--type_info and --stat).
        '''

        X, Y, Z = data[:,1], data[:,2], data[:,3]
        T = data[:,0]

        T = T - T[0]

        dt_array = T[1:]-T[:-1]
        dt, dt_mean, dt_std = dt_array[0]*1e3, dt_array.mean()*1e3, dt_array.std()*1e3

        fig, axes = plt.subplots(3,1)
        plt.subplots_adjust(left=0.07, right=0.9, hspace=0.35, top=0.9, bottom=0.065)
        fig.set_size_inches((11,9))
        fig.suptitle(f"Plot data from file <{self.data_file}>", fontsize=16)
        marker_size = 5 if len(Z) <= 30 else 1

        axe = axes[0]
        axe.set_ylim(-15, 15)
        axe.plot(T, X, '.:b', markersize=marker_size, linewidth=0.3, color='r', label=f"{Plot_IMU.TYPE_INFO[self.type_info][1]} {self.type_info}")
        axe.set_title(f"{Plot_IMU.TYPE_INFO[self.type_info][1]} {Plot_IMU.TYPE_INFO[self.type_info][0]}")
        axe.set_xlabel("Time [s]")
        axe.set_ylabel(fr"{Plot_IMU.TYPE_INFO[self.type_info][0]} [{Plot_IMU.TYPE_INFO[self.type_info][4]}]")
        if self.stat:
            x_mean, x_std = X.mean(), X.std()
            x_min, x_max = X.min(), X.max()
            text1 = f"x_mean: {x_mean*.1:.1f} cm, x_std: {x_std*.1:.1f} cm"
            text1 += f"(min, max): ({x_min*.1:.1f}, {x_max*.1:.1f}) cm"
            text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
            print(text1)
            print(text2)
            box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
            axe.text(0, ymax*.98,
                     fr"mean$_x$: {x_mean*.1:.1f} cm, $\sigma_x$: {x_std*.1:.1f} cm, " +
                     fr"(x$_{{min}}$, $x_{{max}}$): ({x_min*.1:.1f}, {x_max*.1:.1f}) cm"+ "\n" + text2,
                     va='top', ha ='left', fontsize=9, bbox=box)
        axe.legend()
        axe.grid(True)

        axe = axes[1]
        axe.set_ylim(-15, 15)
        axe.plot(T, Y, '.:b', markersize=marker_size, linewidth=0.3, color='g', label=f"{Plot_IMU.TYPE_INFO[self.type_info][2]} {self.type_info}")
        axe.set_title(f"{Plot_IMU.TYPE_INFO[self.type_info][2]} {Plot_IMU.TYPE_INFO[self.type_info][0]}")
        axe.set_xlabel("Time [s]")
        axe.set_ylabel(fr"{Plot_IMU.TYPE_INFO[self.type_info][0]} [{Plot_IMU.TYPE_INFO[self.type_info][4]}]")
        if self.stat:
            y_mean, y_std = Y.mean(), Y.std()
            y_min, y_max = Y.min(), Y.max()
            text1 = f"y_mean: {y_mean*.1:.1f} cm, y_std: {y_std*.1:.1f} cm"
            text1 += f"(min, max): ({y_min*.1:.1f}, {y_max*.1:.1f}) cm"
            text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
            print(text1)
            print(text2)
            box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
            axe.text(0, ymax*.98,
                     fr"mean$_y$: {y_mean*.1:.1f} cm, $\sigma_y$: {y_std*.1:.1f} cm, " +
                     fr"(y$_{{min}}$, $y_{{max}}$): ({y_min*.1:.1f}, {y_max*.1:.1f}) cm"+ "\n" + text2,
                     va='top', ha ='left', fontsize=9, bbox=box)
        axe.legend()
        axe.grid(True)

        axe = axes[2]
        axe.set_ylim(-15, 15)
        axe.plot(T, Z, '.:b', markersize=marker_size, linewidth=0.3, color='b', label=f"{Plot_IMU.TYPE_INFO[self.type_info][3]} {self.type_info}")
        axe.set_title(f"{Plot_IMU.TYPE_INFO[self.type_info][3]} {Plot_IMU.TYPE_INFO[self.type_info][0]}")
        axe.set_xlabel("Time [s]")
        axe.set_ylabel(fr"{Plot_IMU.TYPE_INFO[self.type_info][0]} [{Plot_IMU.TYPE_INFO[self.type_info][4]}]")
        if self.stat:
            z_mean, z_std = Z.mean(), Z.std()
            z_min, z_max = Z.min(), Z.max()
            text1 = f"z_mean: {z_mean*.1:.1f} cm, z_std: {z_std*.1:.1f} cm"
            text1 += f"(min, max): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"
            text2 = f"dt[0]: {dt:.1f} ms (mean, std):({dt_mean:.1f}, {dt_std:.1f}) ms"
            print(text1)
            print(text2)
            box = {'facecolor': (.8,.8,.9,.5) , 'edgecolor':'blue', 'boxstyle': 'square'}
            axe.text(0, ymax*.98,
                     fr"mean$_z$: {z_mean*.1:.1f} cm, $\sigma_z$: {z_std*.1:.1f} cm, " +
                     fr"(z$_{{min}}$, $z_{{max}}$): ({z_min*.1:.1f}, {z_max*.1:.1f}) cm"+ "\n" + text2,
                     va='top', ha ='left', fontsize=9, bbox=box)
        axe.legend()
        axe.grid(True)
        #
        plt.savefig(self.data_file.replace('.txt','.png'))
        plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_file", type=str, default="")
    parser.add_argument('--type_info', type=str, default="",
                         help="Whether to change the name of the figures according to the information given: ('acceleration', 'angular_velocity', 'magnetic' or 'euler')")
    parser.add_argument('--stat', action="store_true",
                         help="wether to compute and plot mean and std")
    args = parser.parse_args()
    data_file = args.data_file
    type_info = args.type_info
    stat = args.stat

    if type_info in Plot_IMU.TYPE_INFO.keys():
        plot = Plot_IMU(type_info, data_file, stat)
        plot.compute()
    else:
        print(f"Error: mode <{type_info}> unknown.\nUse one of {TYPE_INFO.keys()}.")
