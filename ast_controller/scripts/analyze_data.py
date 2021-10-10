#!/usr/bin/env python3
import rospy 
import rospkg 

import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    rospy.init_node("data_plotter")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ast_controller')
    file = open(package_path 
                + '/saved_data/' 
                + 'out_data_1.25.pkl', 'rb')

    data = pkl.load(file)

    offsets = data[0]
    linear_err = data[1]
    angular_err = data[2]
    
    fig = plt.figure(figsize=(15, 15))

    # fig = plt.figure(figsize=(13, 13))
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    
    plt.rcParams.update({'font.size': 14})
    plt.rcParams.update({'axes.titlesize': 'small'})
    # linear_err = np.array(linear_err)*1000
   
    ax1.set_xlabel('Угол поворота (градусы)', fontsize=14)
    ax1.set_ylabel('Ошибка (градусы)', fontsize=14)

    ax2.set_xlabel('Угол поворота (градусы)', fontsize=14)
    ax2.set_ylabel('Ошибка (метры)', fontsize=14)

    # for i in range(len(offsets)):
    medianprops = dict(linestyle='-', linewidth=2.5, color='firebrick')

    ax1.boxplot(angular_err, positions=offsets, showfliers=False, medianprops=medianprops)
    ax2.boxplot(linear_err, positions=offsets, showfliers=False, medianprops=medianprops)
    # ax1.set_xticks(np.arange(offsets[0], offsets, 2.0))
    # ax2.set_xticks(np.arange(min(offsets), max(offsets)+1.0, 2.0)) 
    
    ax1.grid(True, which="both")
    ax2.grid(True, which="both")

    plt.show()

    # try:



    # except rospy.ROSInterruptException:
    #     pass
