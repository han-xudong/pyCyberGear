# -*- coding=utf-8 -*-

# ------------------------------------------------------------------
# File Name:        cybergear_position_mode.py
# Author:           Han Xudong
# Version:          1.0.0
# Created:          2024/06/05
# Description:      A position mode example for CyberGear motors.
#                   The motors will move to the target position, and
#                   the position and velocity of motors will be
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

def cybergear_position_mode(com_port: str,
                            baud_rate: int,
                            ids: list,
                            target_pos: list,
                            vel: int) -> None:
    '''Position mode for CyberGear motors.
    
    Args:
        com_port: The COM port of the CyberGear controller.
        baud_rate: The baud rate of the CyberGear controller.
        ids: The IDs of motors.
        target_pos: The target position of motors.
        vel: The velocity of motors.
        
    Returns:
        None
    '''

    # Create an instance of the CyberGear class
    cybergear = CyberGear(com_port, baud_rate)

    # Set zero position
    for id in ids:
        cybergear.set_zero(id_num=id)

    # Create lists to store the position and velocity of motors
    time_list = [0]
    pos_list = [[0] for i in range(len(ids))]
    vel_list = [[0] for i in range(len(ids))]
    cur_pos = np.zeros(len(ids))

    # Create an interactive plot
    fig = plt.figure(figsize=(10, 5), dpi=100)
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    plt.ion()
    pos_lines = [ax1.plot(pos_list[i], label='Motor ' + ids[i])[0] for i in range(len(ids))]
    vel_lines = [ax2.plot(vel_list[i], label='Motor ' + ids[i])[0] for i in range(len(ids))]
    ax1.legend(loc='upper right')
    ax2.legend(loc='upper right')
    plt.show()

    # Set the start time and error
    start_time = time.time()
    err = 0.2

    # Move motors to the target position
    for id in ids:
        cybergear.move(id_num=id, 
                       pos=target_pos[i], 
                       vel=vel)

    # Display the position and velocity of motors in real time
    while np.mean(np.abs(cur_pos - target_pos)) > err:
        time_list.append(time_list[-1] + 1)
        for i, id in enumerate(ids):
            c_pos, c_vel = cybergear.get_pos_vel(id_num=id)
            pos_list[i].append(c_pos)
            vel_list[i].append(c_vel)
            pos_lines[i].set_data(time_list, pos_list[i])
            vel_lines[i].set_data(time_list, vel_list[i])
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        plt.pause(0.01)

    # Stop the motors
    for id in ids:
        cybergear.stop(id_num=id)
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
               header='Time,' + 
                      ', '.join([f'Motor {id + 1} Position' for id in ids]) +
                      ', '.join([f'Motor {id + 1} Velocity' for id in ids]))
    print('The data is saved as data.csv in ' + save_path)

if __name__ == '__main__':
    # Set the COM port and baud rate of the CyberGear controller
    com_port = 'COM3'
    baud_rate = 115200
    # Set the IDs of motors
    ids = [1, 2]
    # Set the target position of motors
    target_pos = [90, 180]
    # Set the velocity of motors
    vel = 20
    # Run the position mode
    cybergear_position_mode(com_port, 
                            baud_rate, 
                            ids, 
                            target_pos, 
                            vel)