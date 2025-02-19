# -*- coding=utf-8 -*-

# ------------------------------------------------------------------
# File Name:        cybergear_loop.py
# Author:           Han Xudong
# Version:          1.0.0
# Created:          2024/06/05
# Description:      A loop motion example for CyberGear motors.
#                   The motors will move between -360 and 360 degrees
#                   for 10 rounds with a vel of 10 rpm.
#                   The position and velocity of motors will be
#                   displayed in real time.
# ------------------------------------------------------------------

import time
import os
import numpy as np
import matplotlib.pyplot as plt
from pycybergear import CyberGear


def cybergear_loop(
    com_port: str,
    baud_rate: int,
    model: str,
    ids: list,
    pos_range: list,
    rounds: int,
    vel: int,
) -> None:
    """Loop motion for CyberGear motors.

    Args:
        com_port: The COM port of the CyberGear controller.
        baud_rate: The baud rate of the CyberGear controller.
        num: The number of motors.
        pos_range: The angle range of motors.

    Returns:
        None
    """

    # Create an instance of the CyberGear class
    cybergear = CyberGear(com_port, baud_rate, model)

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
    pos_lines = [
        ax1.plot(pos_list[i], label="Motor " + str(ids[i]))[0] for i in range(len(ids))
    ]
    vel_lines = [
        ax2.plot(vel_list[i], label="Motor " + str(ids[i]))[0] for i in range(len(ids))
    ]
    ax1.legend(loc="upper right")
    ax2.legend(loc="upper right")
    plt.show()

    # Set motors move between pos_range for specific rounds with a vel of 10 rpm
    pos_1 = pos_range[0]
    pos_2 = pos_range[1]
    counter = 0
    start_time = time.time()
    err = 0.2

    # Start moving
    print("Start!")
    while counter < rounds:
        # Move to pos_1
        for id in ids:
            cybergear.set_pos(id_num=id, pos=pos_1, vel=vel)
        while np.mean(np.abs(cur_pos - pos_1)) > err:
            time_list.append(time.time() - start_time)
            for i, id in enumerate(ids):
                c_pos, c_vel = cybergear.get_posvel(id_num=id)
                cur_pos[i] = c_pos
                pos_list[i].append(c_pos)
                vel_list[i].append(c_vel)
                pos_lines[i].set_xdata(time_list)
                pos_lines[i].set_ydata(pos_list[i])
                vel_lines[i].set_xdata(time_list)
                vel_lines[i].set_ydata(vel_list[i])
            print(cur_pos, "      ", end="\r")
            ax1.set_xlim([time_list[0], time_list[-1]])
            ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
            ax2.set_xlim([time_list[0], time_list[-1]])
            ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
            ax1.autoscale_view()
            ax2.autoscale_view()
            plt.pause(0.01)

        # Move to pos_2
        for id in ids:
            cybergear.set_pos(id_num=id, pos=pos_2, vel=vel)
        while np.mean(np.abs(cur_pos - pos_2)) > err:
            time_list.append(time.time() - start_time)
            for i, id in enumerate(ids):
                c_pos, c_vel = cybergear.get_posvel(id_num=id)
                cur_pos[i] = c_pos
                pos_list[i].append(c_pos)
                vel_list[i].append(c_vel)
                pos_lines[i].set_xdata(time_list)
                pos_lines[i].set_ydata(pos_list[i])
                vel_lines[i].set_xdata(time_list)
                vel_lines[i].set_ydata(vel_list[i])
            print(cur_pos, "      ", end="\r")
            ax1.set_xlim([time_list[0], time_list[-1]])
            ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
            ax2.set_xlim([time_list[0], time_list[-1]])
            ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
            ax1.autoscale_view()
            ax2.autoscale_view()
            plt.pause(0.01)
        counter += 1

    # Back to zero
    pos_0 = 0
    for id in ids:
        cybergear.set_pos(id_num=id, pos=pos_0, vel=vel)
    while np.mean(np.abs(cur_pos - pos_0)) > err:
        time_list.append(time.time() - start_time)
        for i, id in enumerate(ids):
            c_pos, c_vel = cybergear.get_posvel(id_num=id)
            cur_pos[i] = c_pos
            pos_list[i].append(c_pos)
            vel_list[i].append(c_vel)
            pos_lines[i].set_xdata(time_list)
            pos_lines[i].set_ydata(pos_list[i])
            vel_lines[i].set_xdata(time_list)
            vel_lines[i].set_ydata(vel_list[i])
        print(cur_pos, "      ", end="\r")
        ax1.set_xlim([time_list[0], time_list[-1]])
        ax1.set_ylim([np.min(pos_list), np.max(pos_list)])
        ax2.set_xlim([time_list[0], time_list[-1]])
        ax2.set_ylim([np.min(vel_list), np.max(vel_list)])
        ax1.autoscale_view()
        ax2.autoscale_view()
        plt.pause(0.01)

    # Stop the motors
    for id in ids:
        cybergear.motor_stop(id_num=id)
    print("\nDone!")

    # Save the figure and the data
    save_path = time.strftime("%Y%m%d%H%M%S", time.localtime()) + "/"
    os.makedirs(save_path)
    fig.savefig(save_path + "curve.png")
    print("The figure is saved as curve.png in " + save_path)
    np.savetxt(
        save_path + "data.csv",
        np.array([time_list] + pos_list + vel_list).T,
        delimiter=",",
        fmt="%.3f",
        header="Time"
        + ", ".join([f"Motor {id + 1} Position" for id in ids])
        + ", ".join([f"Motor {id + 1} Velocity" for id in ids]),
    )
    print("The data is saved as data.csv in " + save_path)


if __name__ == "__main__":
    # Set the COM port and baud rate
    com_port = "COM11"
    baud_rate = 115200
    model = "CAN"
    # Set the ids of motors
    ids = [7]
    # Set the angle range
    pos_range = [-360, 360]
    # Set the number of rounds
    rounds = 10
    # Set the velocity
    vel = 20
    # Start the loop motion
    cybergear_loop(com_port, baud_rate, model, ids, pos_range, rounds, vel)
