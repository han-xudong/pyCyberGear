# -*- coding=utf-8 -*-

# ------------------------------------------------------------------
# File Name:        cybergear_loop.py
# Author:           Han Xudong
# Version:          1.0.0
# Created:          2024/06/05
# Description:      A loop motion example for CyberGear motors.
#                   The motors will move between -360 and 360 degrees 
#                   for 10 rounds with a speed of 10 rpm.
#                   The position and velocity of motors will be
#                   displayed in real time.
# Function List:    cybergear_loop
# History:
#       <author>        <version>       <time>      <desc>
#       Han Xudong      1.0.0           2024/06/21  Created the module
# ------------------------------------------------------------------

import time
import os
import numpy as np
import matplotlib.pyplot as plt
from pycybergear import CyberGear

def cybergear_loop(com_port: str, 
                   baud_rate: int, 
                   num: int,
                   angle_range: list) -> None:
    '''Loop motion for CyberGear motors.

    Args:
        com_port: The COM port of the CyberGear controller.
        baud_rate: The baud rate of the CyberGear controller.
        num: The number of motors.
        angle_range: The angle range of motors.

    Returns:
        None
    '''
    
    # Create an instance of the CyberGear class
    cybergear = CyberGear(com_port, baud_rate)

    # Set zero position
    for i in range(1, num + 1):
        cybergear.set_zero(id_num=i)

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
    angle_1 = angle_range[0]
    angle_2 = angle_range[1]
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
            cybergear.set_pos(id_num=i + 1, angle=angle_1, speed=speed)
        while np.mean(np.abs(cur_pos - angle_1)) > acc:
            time_list.append(time.time() - start_time)
            for i in range(num):
                pos, vel = cybergear.get_posvel(id_num=i + 1)
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
            cybergear.set_pos(id_num=i + 1, angle=angle_2, speed=speed)
        while np.mean(np.abs(cur_pos - angle_2)) > acc:
            time_list.append(time.time() - start_time)
            for i in range(num):
                pos, vel = cybergear.get_posvel(id_num=i + 1)
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
        cybergear.set_pos(id_num=i + 1, angle=angle_0, speed=speed)
    while np.mean(np.abs(cur_pos - angle_0)) > acc:
        time_list.append(time.time() - start_time)
        for i in range(num):
            pos, vel = cybergear.get_posvel(id_num=i + 1)
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
        cybergear.motor_estop(id_num=i + 1)
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
    # Set the COM port and baud rate
    com_port = 'COM3'
    baud_rate = 115200
    # Set the number of motors
    num = 2
    # Set the angle range
    angle_range = [-360, 360]
    # Start the loop motion
    cybergear_loop(com_port, 
                   baud_rate,
                   num,
                   angle_range)