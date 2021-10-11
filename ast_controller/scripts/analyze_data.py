#!/usr/bin/env python3
import rospy 
import rospkg 

import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np

import glob
import os


if __name__ == '__main__':
    rospy.init_node("data_plotter")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ast_controller')
    data_path = package_path + '/saved_data/'

    list_of_files = glob.glob(data_path + '*')
    latest_file = max(list_of_files, key=os.path.getctime)


    file = open(latest_file, 'rb')

    data = pkl.load(file)

    offsets = data[0]
    linear_err = data[1]
    angular_err = data[2]
    
    fig = plt.figure(figsize=(15, 15))

    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    
    plt.rcParams.update({'font.size': 14})
    plt.rcParams.update({'axes.titlesize': 'small'})
   
    ax1.set_xlabel('Rotation (degrees)', fontsize=14)
    ax1.set_ylabel('Error (degrees)', fontsize=14)

    ax2.set_xlabel('Rotation (degrees)', fontsize=14)
    ax2.set_ylabel('Error (metres)', fontsize=14)

    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')

    ax1.boxplot(angular_err, positions=offsets, showfliers=False, medianprops=medianprops)
    ax2.boxplot(linear_err, positions=offsets, showfliers=False, medianprops=medianprops)
    
    ax1.grid(True, which="both")
    ax2.grid(True, which="both")

    plt.show()

