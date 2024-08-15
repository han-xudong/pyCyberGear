# -*- coding=utf-8 -*-

# ------------------------------------------------------------------
# File Name:        test.py
# Author:           Han Xudong
# Version:          1.0.0
# Created:          2024/06/05
# Description:      Test the CyberGear module.
#                   The motors will move between 0 and 360 degrees 
#                   for 10 rounds with a speed of 10 rpm.
#                   The position and velocity of motors will be
#                   displayed in real time, and the figure will be
#                   saved as test_result.png after the test.
# Function List:    cybergear_test
# History:
#       <author>        <version>       <time>      <desc>
#       Han Xudong      1.0.0           2024/06/21  Created the module
# ------------------------------------------------------------------

from pycybergear.CyberGear import CyberGear
import time
import os
import numpy as np
import matplotlib.pyplot as plt

def cybergear_test(com_port, baud_rate, num):
    # Create an instance of the CyberGear class
    cyber_gear = CyberGear(com_port, baud_rate)

    # Set zero position
    # for i in range(1, num + 1):
    #     cyber_gear.set_zero_position(id_num=i)

    # Create lists to store the position and velocity of motors
    time_list = [0]
    pos_list = [[0] for i in range(num)]
    vel_list = [[0] for i in range(num)]
    cur_pos = np.zeros(num)

    # Create an interactive plot
    fig = plt.figure(figsize=(10, 5), dpi=100)
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    plt.ion()
    pos_lines = [ax1.plot(pos_list[i], label='Motor ' + str(i + 1))[0] for i in range(num)]
    vel_lines = [ax2.plot(vel_list[i], label='Motor ' + str(i + 1))[0] for i in range(num)]
    ax1.legend(loc='upper right')
    ax2.legend(loc='upper right')
    plt.show()

    # Set motors move between 0 and 360 degrees for 10 rounds with a speed of 10 rpm
    angle_1 = -360
    angle_2 = 360
    rounds = 10
    speed = 20
    gap_time = 0.1
    counter = 0
    start_time = time.time()
    acc = 0.2
    # Start moving
    print('Start!')
    while counter < rounds:
        for i in range(num):
            cyber_gear.set_angle(id_num=i + 1, angle=angle_1, speed=speed)
        while np.mean(np.abs(cur_pos - angle_1)) > acc:
            time_list.append(time.time() - start_time)
            for i in range(num):
                pos, vel = cyber_gear.get_state(id_num=i + 1)
                cur_pos[i] = pos
                pos_list[i].append(pos)
                vel_list[i].append(vel)
                pos_lines[i].set_xdata(time_list)
                pos_lines[i].set_ydata(pos_list[i])
                vel_lines[i].set_xdata(time_list)
                vel_lines[i].set_ydata(vel_list[i])
            print(cur_pos, '      ', end='\r')
            ax1.set_xlim([time_list[0], time_list[-1]])
            ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
            ax2.set_xlim([time_list[0], time_list[-1]])
            ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
            ax1.autoscale_view()
            ax2.autoscale_view()
            plt.pause(0.01)
        for i in range(num):
            cyber_gear.set_angle(id_num=i + 1, angle=angle_2, speed=speed)
        while np.mean(np.abs(cur_pos - angle_2)) > acc:
            time_list.append(time.time() - start_time)
            for i in range(num):
                pos, vel = cyber_gear.get_state(id_num=i + 1)
                cur_pos[i] = pos
                pos_list[i].append(pos)
                vel_list[i].append(vel)
                pos_lines[i].set_xdata(time_list)
                pos_lines[i].set_ydata(pos_list[i])
                vel_lines[i].set_xdata(time_list)
                vel_lines[i].set_ydata(vel_list[i])
            print(cur_pos, '      ', end='\r')
            ax1.set_xlim([time_list[0], time_list[-1]])
            ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
            ax2.set_xlim([time_list[0], time_list[-1]])
            ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
            ax1.autoscale_view()
            ax2.autoscale_view()
            plt.pause(0.01)
        counter += 1

    # Back to zero
    angle_0 = 0
    for i in range(num):
        cyber_gear.set_angle(id_num=i + 1, angle=angle_0, speed=speed)
    while np.mean(np.abs(cur_pos - angle_0)) > acc:
        time_list.append(time.time() - start_time)
        for i in range(num):
            pos, vel = cyber_gear.get_state(id_num=i + 1)
            cur_pos[i] = pos
            pos_list[i].append(pos)
            vel_list[i].append(vel)
            pos_lines[i].set_xdata(time_list)
            pos_lines[i].set_ydata(pos_list[i])
            vel_lines[i].set_xdata(time_list)
            vel_lines[i].set_ydata(vel_list[i])
        print(cur_pos, '      ', end='\r')
        ax1.set_xlim([time_list[0], time_list[-1]])
        ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
        ax2.set_xlim([time_list[0], time_list[-1]])
        ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
        ax1.autoscale_view()
        ax2.autoscale_view()
        plt.pause(0.01)

    # Stop moving
    for i in range(num):
        cyber_gear.motor_estop(id_num=i + 1)
    print('\nDone!')

    # Save the figure and the data
    save_path = time.strftime('%Y%m%d%H%M%S', time.localtime()) + '/'
    os.makedirs(save_path)
    fig.savefig(save_path + 'curve.png')
    print('The figure is saved as curve.png in ' + save_path)
    np.savetxt(save_path + 'data.csv', 
               np.array([time_list] + pos_list + vel_list).T, 
               delimiter=',', 
               fmt='%.3f', 
               header='Time' + 
                      ', '.join([f'Motor {i + 1} Position' for i in range(num)]) + 
                      ', '.join([f'Motor {i + 1} Velocity' for i in range(num)]))
    print('The data is saved as data.csv in ' + save_path)

if __name__ == '__main__':
    com_port = 'COM3'
    baud_rate = 115200
    num = 2
    cybergear_test(com_port, 
                   baud_rate,
                   num)