# -*- coding=utf-8 -*-

# ------------------------------------------------------------------
# File Name:        CyberGear.py
# Author:           Han Xudong
# Version:          1.0.1
# Created:          2024/06/05
# Description:      Xiaomi CyberGear Micro Motor Python Control Library.
# Function List:    CyberGear: Class of Xiaomi CyberGear Micro Motor.
#                   _write_port: Write data to serial port.
#                   _read_port: Read data from serial port.
#                   _can_to_uart: USB to CAN module packet mode: CAN message -> Serial frame.
#                   _uart_to_can: USB to CAN module packet mode: Serial frame -> CAN message.
#                   _send_can: CAN send function.
#                   _receive_can: CAN receive function.
#                   _format_data: Data format conversion function.
#                   _float_to_uint: Float to uint conversion.
#                   _uint_to_float: Uint to float conversion.
#                   _dump_error: Fault feedback frame parsing.
#                   _reply_state: Motor response feedback frame.
#                   _write_prop: Modify motor attribute parameters.
#                   _read_prop: Read motor attribute parameters.
#                   motor_enable: Motor enable.
#                   motor_stop: Stop running.
#                   set_mode: Set motor mode.
#                   impedance_control: Impedance control mode.
#                   set_pos: Position control.
#                   set_vel: Speed control.
#                   set_torque: Torque (current) control.
#                   set_zero: Set zero position of the motor.
#                   set_id: Set motor ID number.
#                   clear_error: Clear motor error.
#                   restore_config: Restore factory settings.
#                   get_id: Get motor ID number.
#                   get_posvel: Get motor position and speed.
#                   get_volcur: Get motor voltage and current.
# History:
#       <author>        <version>       <time>      <desc>
#       Han Xudong      1.0.0           2024/06/05  Created the module
#       Han Xudong      1.0.1           2024/08/15  Added description
# ------------------------------------------------------------------

import time
import serial
import math as cm
import struct
import numpy as np

class CyberGear():
    def __init__(self,
                 com_port='COM3',
                 baud_rate=115200,
                 model="DR") -> None:
        '''Xiaomi CyberGear Micro Motor Python Control Library,
        which can be used to control Xiaomi CyberGear micro motors
        through serial port communication.

        Args:
            com_port: Serial port number
            baud_rate: Baud rate (default is 115200)

        Returns:
            None
        '''

        # Motor control parameters
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0
        self.T_MIN = -12.0
        self.T_MAX = 12.0
        self.I_MAX = 27
        self.TORQUE_CONSTANT = self.T_MAX / self.I_MAX
        self.RAD_DEG = 180 / cm.pi
        self.DEG_RAD = cm.pi / 180
        self.RAD_S_R_MIN = 30 / cm.pi
        self.R_MIN_RAD_S = cm.pi / 30

        # Test motor control under Windows, 
        # corresponding to the connected COM port and baud rate
        self.uart = serial.Serial(com_port, baud_rate)
        self.model = model

        # For Linux,
        # Test under Jetson Nano (Ubuntu) and Raspberry Pi (Raspbian), 
        # corresponding to the connected serial port and baud rate

        # result = os.popen("sudo ls -l /dev/ttyACM*").read()

        # sudo chmod 666 /dev/ttyACM* sometimes there will be an error, 
        # showing no curve to open the serial port, 
        # then you need to run the command on the left

        # com = result.split()[-1]
        # os.system("sudo chmod 777 " + com)
        # # os.system("sudo fuser -k " + com)
        # uart = serial.Serial(com, 115200, timeout=0.5)

        self.ERROR_FLAG = "Status Normal"
        self.READ_FLAG = 0  # read flag
        self.MOTOR_NUM = 127  # motor number
        self.MCU_ID = []

        # Motor status two-dimensional array, 
        # get the real-time return status 
        # [position,speed,torque,motor_temp,axis_error,mode_status] of the motor id_num 
        # through motor_state[id_num-1], 
        # the units are degree, r/min, Nm, 
        # the three variable values refer to the motor output shaft,
        # where motor_state[id_num-1][0] represents the position of the id_num motor, 
        # motor_state[id_num-1][1] represents the speed of the id_num motor, 
        # motor_state[id_num-1][2] represents the output torque of the id_num motor
        self.motor_state = np.zeros((self.MOTOR_NUM, 6))

    def _write_port(self, 
                    data=[]):
        '''Write data to serial port.
        
        Args:
            data: Data to be written
            
        Returns:
            result: Write result

        Raises:
            "!!!ERROR IN WRITING DATA"
        '''

        # Clear the serial port buffer
        if self.uart.inWaiting() > 0:
            self.uart.read(self.uart.inWaiting())
        # Write data to the serial port
        try:
            result = self.uart.write(data)
            return result
        except Exception as e:
            print("!!!ERROR IN WRITING DATA:", e)
            print("Restart the serial port")
            self.uart.close()
            self.uart.open()
            result = self.uart.write(data)
            return result

    def _read_port(self, 
                   num=16):
        '''Read data from serial port.

        Args:
            num: Number of bytes to be read

        Returns:
            byte_list: List of bytes read
        '''

        # Read data from the serial port
        self.READ_FLAG = -1
        byte_list = []
        i = 5000
        while self.uart.inWaiting() == 0 and i > 0:
            i -= 1
        # time.sleep(0.001)
        while self.uart.inWaiting() > 0:
            byte_list.append(list(self.uart.read(1))[0])
        
        print(byte_list)
        # Return the received data
        if len(byte_list) == num:
            self.READ_FLAG = 1
            return byte_list
        else:
            print("Received data error in _read_port():" + str(byte_list))
            self.READ_FLAG = -1
            return

    def _can_to_uart(self, 
                     data=[], 
                     rtr=0):
        '''USB to CAN module packet mode: CAN message -> Serial frame.
        
        Args:
            data: CAN message data
            rtr: Remote frame flag
            
        Returns:
            udata: Serial frame data
        '''


        if self.model == "DR":
            udata = [0xAA, 1, 0, 0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            if len(data) == 13 and data[0] == 0x08:
                for i in range(12):
                    udata[4 + i] = data[i + 1]
                return udata
            else:
                return []
        elif self.model == "CAN":
            udata = [0x41, 0x54, 0x0, 0x0, 0x0, 0x0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a]
            if len(data) == 13 and data[4] == 0x08:
                for i in range(13):
                    udata[2 + i] = data[i]
            return udata
        else:
            return []

    def _uart_to_can(self, 
                     data=[]):
        '''USB to CAN module packet mode: Serial frame -> CAN message.

        Args:
            data: Serial frame data

        Returns:
            cdata: CAN message data
        '''
        if self.model == "DR":
            cdata = [0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            if len(data) == 16 and data[3] == 0x08:
                for i in range(12):
                    cdata[1 + i] = data[i + 4]
                return cdata
            else:
                self.READ_FLAG = -1
                return []
        elif self.model == "CAN":
            cdata = [0x0, 0x0, 0x0, 0x0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            if len(data) == 17 and data[6] == 0x08:
                for i in range(13):
                    cdata[i] = data[i + 2]
                return cdata
            else:
                self.READ_FLAG = -1
                return []

    def _send_can(self, 
                  id_num=127, 
                  cmd_mode=0, 
                  cmd_data=[], 
                  data=[], 
                  rtr=0):
        '''CAN send function.
        
        Args:
            id_num: Motor ID number
            cmd_mode: Command mode
            cmd_data: Command data
            data: Data
            rtr: Remote frame flag
            
        Returns:
            None
        '''

        if self.model == "DR":
            cdata = [0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            cdata[1] = cmd_mode
            cdata[2] = cmd_data[1]
            cdata[3] = cmd_data[0]  # master_ID
            cdata[4] = int(id_num)
        elif self.model == "CAN":
            cdata = [0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            cdata[0] = (cmd_mode << 3) | (cmd_data[1] >> 5)
            cdata[1] = (cmd_data[1] << 3) | (cmd_data[0] >> 5)
            cdata[2] = ((cmd_data[0] << 3) | (id_num >>5)) & 0xff
            cdata[3] = (id_num << 3) | 0x04
        for i in range(8):
            cdata[5 + i] = data[i]
        self._write_port(data=self._can_to_uart(data=cdata, 
                                                rtr=rtr))

    def _receive_can(self):
        '''CAN receive function.

        Args:
            None

        Returns:
            cdata: CAN message data
        '''

        udata = self._read_port(16)
        if self.READ_FLAG == 1:
            cdata = self._uart_to_can(data=udata)
            if cdata[1] == 21:
                self._dump_error(cdata, True)
            return cdata[:]

    def _format_data(self, 
                     data=[], 
                     format="f f", 
                     type='decode'):
        '''Data format conversion function.
        Decode is to convert binary (bytes) into data that people can understand,
        while encode is the opposite.

        Args:
            data: Data to be converted
            format: Data format
            type: Conversion type ('decode' or 'encode')

        Returns:
            rdata: Converted data
        '''

        format_list = format.split()
        rdata = []
        if type == 'decode':
            p = 0
            for f in format_list:
                s_f = []
                if f == 'f':
                    s_f = [4, 'f']
                elif f == 'u16':
                    s_f = [2, 'H']
                elif f == 's16':
                    s_f = [2, 'h']
                elif f == 'u32':
                    s_f = [4, 'I']
                elif f == 's32':
                    s_f = [4, 'i']
                elif f == 'u8':
                    s_f = [1, 'B']
                elif f == 's8':
                    s_f = [1, 'b']
                ba = bytearray()
                if len(s_f) == 2:
                    for i in range(s_f[0]):
                        ba.append(data[p])
                        p = p + 1
                    rdata.append(struct.unpack(s_f[1], ba)[0])
                else:
                    print('Unknown format in _format_data(): ' + f)
                    return []
            return rdata
        elif type == 'encode' and len(format_list) == len(data):
            for i in range(len(format_list)):
                f = format_list[i]
                s_f = []
                if f == 'f':
                    s_f = [4, 'f']
                elif f == 'u16':
                    s_f = [2, 'H']
                elif f == 's16':
                    s_f = [2, 'h']
                elif f == 'u32':
                    s_f = [4, 'I']
                elif f == 's32':
                    s_f = [4, 'i']
                elif f == 'u8':
                    s_f = [1, 'B']
                elif f == 's8':
                    s_f = [1, 'b']
                if f != 'f':
                    data[i] = int(data[i])
                if len(s_f) == 2:
                    bs = struct.pack(s_f[1], 
                                     data[i])
                    for j in range(s_f[0]):
                        rdata.append(bs[j])
                else:
                    print('Unknown format in _format_data(): ' + f)
                    return []
            if len(rdata) < 4:
                for i in range(4 - len(rdata)):
                    rdata.append(0x00)
            return rdata


    def _float_to_uint(self, 
                       x, 
                       x_min, 
                       x_max, 
                       bits):
        '''Float to uint conversion.
        
        Args:
            x: Float data
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits
            
        Returns:
            rdata: Converted data
        '''

        span = x_max - x_min
        offset = x_min
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int(((x - offset)*((1 << bits) - 1)/span))

    def _uint_to_float(self, 
                       x, 
                       x_min, 
                       x_max, 
                       bits):
        '''Uint to float conversion.

        Args:
            x: Uint data
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits

        Returns:
            rdata: Converted data
        '''

        span = (1 << bits) - 1
        offset = x_max - x_min
        if x > span:
            x = span
        elif x < 0:
            x = 0
        return offset*x/span + x_min

    def _dump_error(self, 
                    rdata=[]):
        '''
        Print motor error number (motor automatically returns error information)
        Read the motor error information code, 
        if the error code is 0, it means there is no exception;
        if the error code is not 0, it means there is a fault
        
        Args:
            rdata: Fault feedback frame content
        
        Returns:
            None
        '''

        # [0x08 mode cmd_data[1] cmd_data[0] id_num data0 data1 data2 data3 data4 data5 data6 data7]
        mode = rdata[1]
        cmd_data = rdata[2:4]
        id_num = rdata[4]
        data = rdata[5:]
        if self.READ_FLAG == 1 and mode == 21:
            print("Motor CAN_ID" + str(id_num))
            print("Main CAN_ID" + str(cmd_data[0]))
        if data[0] & 0x3F or data[5] & (0x01 << 0):
            self.ERROR_FLAG = "Status abnormal:"
            if data[0] & (0x01 << 0):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Motor over temperature fault, default is 80 C"
            if data[0] & (0x01 << 1):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Drive chip fault"
            if data[0] & (0x01 << 2):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Voltage fault"
            if data[0] & (0x01 << 3):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Overvoltage fault"
            if data[0] & (0x01 << 4):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Phase B current sampling overcurrent"
            if data[0] & (0x01 << 5):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Phase C current sampling overcurrent"
            if data[0] & (0x01 << 7):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Encoder not calibrated"
            if data[1] & 0xFF:
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Overload fault"
            if data[2] & (0x01 << 0):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Phase A current sampling overcurrent"
            if data[4] & (0x01 << 0):
                self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Motor over temperature warning, default is 75 C"
        else:
            self.ERROR_FLAG = "Status normal"
        print(self.ERROR_FLAG)
        return self.ERROR_FLAG

    def _reply_state(self, 
                     id_num=127):
        '''Motor motion control command real-time return parameters
        This function reads the real-time return parameters 
        [position, speed, torque, temp, error_flag, mode_status] 
        of the motor motion control command, with units of degree, r/min, Nm, respectively, 
        all referring to the motor output shaft.
        Where motor_state[id_num-1][0] represents the position of the id_num motor, 
        motor_state[id_num-1][1] represents the speed of the id_num motor, 
        motor_state[id_num-1][2] represents the output torque of the id_num motor.
        
        Args:
            id_num The motor number to be read. Note that this command id_num cannot be 0.

        Returns:
            None

        Raises:
            "!!!ERROR IN _reply_state"
        '''

        try:
            if id_num <= self.MOTOR_NUM:
                self.READ_FLAG = 0
                rdata = self._receive_can()
                # [0x08 mode cmd_data[1] cmd_data[0] id_num data0 data1 data2 data3 data4 data5 data6 data7]
                if self.READ_FLAG == 1 and rdata[1] == 2:
                    cmd_data = [rdata[3], rdata[2]]
                    id_num = rdata[3]
                    data = rdata[5:]
                    self.motor_state[id_num - 1][0] = self._uint_to_float((data[0] << 8) + data[1], 
                                                                          self.P_MIN, self.P_MAX, 16)*self.RAD_DEG
                    self.motor_state[id_num - 1][1] = self._uint_to_float((data[2] << 8) + data[3], 
                                                                          self.V_MIN, self.V_MAX, 16)*self.RAD_S_R_MIN
                    self.motor_state[id_num - 1][2] = self._uint_to_float((data[4] << 8) + data[5], 
                                                                          self.T_MIN, self.T_MAX, 16)
                    self.motor_state[id_num - 1][3] = ((data[6] << 8) + data[7])*0.1
                    if cmd_data[1] & 0x3F:
                        self.motor_state[id_num - 1][4] = 1
                        self.ERROR_FLAG = 'Status abnormal: '
                        if cmd_data[1] & (0x01 << 0):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Voltage fault"
                        if cmd_data[1] & (0x01 << 1):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Overcurrent fault"
                        if cmd_data[1] & (0x01 << 2):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Overtemperature fault"
                        if cmd_data[1] & (0x01 << 3):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Encoder fault"
                        if cmd_data[1] & (0x01 << 4):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "HALL fault"
                        if cmd_data[1] & (0x01 << 5):
                            self.ERROR_FLAG = self.ERROR_FLAG + '\n' + "Encoder not calibrated"
                        print(self.ERROR_FLAG)
                    else:
                        self.ERROR_FLAG = 'Status normal'
                        self.motor_state[id_num - 1][4] = 0
                    mode_status = (cmd_data[1] >> 6) & 0x03
                    self.motor_state[id_num - 1][5] = mode_status
        except Exception as e:
            print("!!!ERROR IN _reply_state:", e)

    def _write_prop(self, 
                    id_num=127, 
                    index=0, 
                    data_type='f', 
                    value=0):
        '''Modify motor attribute parameters

        Args:
            id_num: The ID number of the motor to be modified
            index: The address of the parameter to be read
            data_type: The data type of the parameter to be written: 
                       'f':float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'
            value: Corresponding parameter data.

        Returns:
            None
        '''
        
        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        tx_data[0] = index & 0xFF
        tx_data[1] = (index >> 8) & 0xFF
        cmd_mode = 18
        if index < 0x7000:
            cmd_mode = 8
            type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
            tx_data[2] = type_list.index(data_type)
        tx_data[4:] = self._format_data(data=[value], 
                                        format=data_type, 
                                        type="encode")
        # Need to send using extended frame (data frame)
        self._send_can(id_num=id_num, 
                       cmd_mode=cmd_mode, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        self._reply_state(id_num=id_num)
        # Save forever
        if cmd_mode == 8:
            self.motor_stop(id_num=id_num)
            cmd_data[1] = 0x02
            tx_data = [0]*8
            # Need to send using extended frame (data frame)
            self._send_can(id_num=id_num, 
                           cmd_mode=cmd_mode, 
                           cmd_data=cmd_data, 
                           data=tx_data, 
                           rtr=0)
            time.sleep(0.1)
            self._reply_state(id_num=id_num)

    def _read_prop(self, 
                   id_num=127, 
                   index=0, 
                   data_type='f'):
        '''Read motor attribute parameters

        Args:
            id_num: The ID number of the motor to be read
            index: The address of the parameter to be read
            data_type: The data type of the parameter to be read: 
                       'f':float,'u16':uint16,'s16':int16,'u32':uint32,'s32':int32,'u8':uint8,'s8':'int8'

        Returns:
            value: Returns the value of the corresponding attribute parameter
        '''

        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        tx_data[0] = index & 0xFF
        tx_data[1] = (index >> 8) & 0xFF
        cmd_mode = 17
        if index < 0x7000:
            cmd_mode = 9
            type_list = ['u8', 's8', 'u16', 's16', 'u32', 's32', 'f']
            tx_data[2] = type_list.index(data_type)
        # Need to send using extended frame (data frame)
        self._send_can(id_num=id_num, 
                       cmd_mode=cmd_mode, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        data = self._receive_can()
        if READ_FLAG == 1 and (data[1] == 17 or data[1] == 9):
            value = self._format_data(data=data[9:], 
                                      format=data_type, 
                                      type="decode")
            return value[0]

    def motor_enable(self, 
                     id_num=127):
        '''Motor enable function

        Args:
            id_num: The ID number of the motor to be enabled

        Returns:
            None
        '''

        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        for i in range(8):
            tx_data[i] = 0x00
        
        self._send_can(id_num=id_num, 
                       cmd_mode=3, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        self._reply_state(id_num=id_num)

    def motor_stop(self, 
                    id_num=127):
        '''Stop running function

        Args:
            id_num: The ID number of the motor to be stopped

        Returns:
            None
        '''

        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        for i in range(8):
            tx_data[i] = 0x00

        self._send_can(id_num=id_num, 
                       cmd_mode=4, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        self._reply_state(id_num=id_num)

    def set_mode(self, 
                 id_num=127, 
                 mode=0):
        '''Set the motor to enter different control modes

        Args:
            id_num: The ID number of the motor to be set
            mode: Motor mode number
                mode = 0: Impedance control mode
                mode = 1: Position mode
                mode = 2: Speed mode
                mode = 3: Torque mode

        Returns:
            None
        '''

        self._write_prop(id_num=id_num, 
                         index=0x7005, 
                         value=mode, 
                         data_type='u8')

    def impedance_control(self, 
                          id_num=127, 
                          pos=0, 
                          vel=0, 
                          tff=0, 
                          kp=0, 
                          kd=0):
        '''Impedance control mode.

        Args:
            id_num: The ID number of the motor to be set
            pos: Motor target position (degrees)
            vel: Motor target speed (r/min)
            tff: Feedforward torque (Nm)
            kp: Stiffness coefficient (rad/Nm)
            kd: Damping coefficient (rad/s/Nm)

        Returns:
            None

        Raises:
            "!!!ERROR IN IMPENDENCE CONTROL"
        '''

        try:
            self.motor_enable(id_num=id_num)
            self.set_mode(id_num=id_num, 
                          mode=0)
            cmd_data = [0]*2
            cmd_data[0] = (self._float_to_uint(tff, 
                                               self.T_MIN, 
                                               self.T_MAX, 
                                               16)) & 0xFF
            cmd_data[1] = ((self._float_to_uint(tff, 
                                                self.T_MIN, 
                                                self.T_MAX, 
                                                16)) >> 8) & 0xFF
            tx_data = [0]*8
            tx_data[0] = (self._float_to_uint(pos*self.DEG_RAD, 
                                              self.P_MIN, 
                                              self.P_MAX, 
                                              16) >> 8) & 0xFF
            tx_data[1] = (self._float_to_uint(pos*self.DEG_RAD, 
                                              self.P_MIN, 
                                              self.P_MAX, 
                                              16)) & 0xFF
            tx_data[2] = (self._float_to_uint(vel*self.R_MIN_RAD_S, 
                                              self.V_MIN, 
                                              self.V_MAX, 
                                              16) >> 8) & 0xFF
            tx_data[3] = (self._float_to_uint(vel*self.R_MIN_RAD_S, 
                                              self.V_MIN, 
                                              self.V_MAX, 
                                              16)) & 0xFF
            tx_data[4] = (self._float_to_uint(kp, 
                                              self.KP_MIN, 
                                              self.KP_MAX, 
                                              16) >> 8) & 0xFF
            tx_data[5] = (self._float_to_uint(kp, 
                                              self.KP_MIN, 
                                              self.KP_MAX, 
                                              16)) & 0xFF
            tx_data[6] = (self._float_to_uint(kd, 
                                              self.KD_MIN, 
                                              self.KD_MAX, 
                                              16) >> 8) & 0xFF
            tx_data[7] = (self._float_to_uint(kd, 
                                              self.KD_MIN, 
                                              self.KD_MAX, 
                                              16)) & 0xFF
            # Need to send using extended frame (data frame)
            self._send_can(id_num=id_num, 
                           cmd_mode=1, 
                           cmd_data=cmd_data, 
                           data=tx_data, 
                           rtr=0)
            self._reply_state(id_num=id_num)
        except Exception as e:
            print("!!!ERROR IN IMPENDENCE CONTROL:", e)

    def set_pos(self, 
                id_num=127, 
                pos=0, 
                vel=10, 
                limit_cur=27):
        '''Motor position control function.
        Control the specified motor to rotate to 
        the specified position at the specified speed

        Args:
            id_num: The ID number of the motor to be set
            pos: Motor position (degrees)
            vel: Maximum speed limit or feedforward speed (0~300r/min)
            limit_cur: Current limit (0-27A)

        Returns:
            None
        '''

        self.motor_enable(id_num=id_num)
        self.set_mode(id_num=id_num, 
                      mode=1)
        self._write_prop(id_num=id_num, 
                         index=0x7018, 
                         value=limit_cur, 
                         data_type='f')
        self._write_prop(id_num=id_num, 
                         index=0x7017, 
                         value=vel*self.R_MIN_RAD_S, 
                         data_type='f')
        self._write_prop(id_num=id_num, 
                         index=0x7016, 
                         value=pos*self.DEG_RAD, 
                         data_type='f')

    def set_vel(self, 
                id_num=127, 
                vel=10, 
                limit_cur=27):
        '''Motor speed control function.
        Control the specified motor to 
        continuously rotate at the specified speed.

        Args:
            id_num: The ID number of the motor to be set
            vel: Target speed (-300~300r/min)
            limit_cur: Current limit (0-27A)

        Returns:
            None
        '''

        self.motor_enable(id_num=id_num)
        self.set_mode(id_num=id_num, 
                      mode=2)
        self._write_prop(id_num=id_num, 
                         index=0x7018, 
                         value=limit_cur, 
                         data_type='f')
        self._write_prop(id_num=id_num, 
                         index=0x700A, 
                         value=vel*self.R_MIN_RAD_S, 
                         data_type='f')

    def set_torque(self, 
                   id_num=127, 
                   torque=0.1):
        '''Motor torque (current) closed-loop control function.
        Control the specified motor to output the specified torque (Nm)

        Args:
            id_num: The ID number of the motor to be set
            torque: Motor output (0~12Nm)

        Returns:
            None
        '''

        self.motor_enable(id_num=id_num)
        self.set_mode(id_num=id_num, 
                      mode=3)
        self._write_prop(id_num=id_num, 
                         index=0x7006, 
                         value=torque/self.TORQUE_CONSTANT, 
                         data_type='f')

    def set_zero(self, 
                 id_num=127):
        '''Set motor zero position function

        Args:
            id_num: The ID number of the motor to be set

        Returns:
            None
        '''

        mode_status = self.motor_state[id_num - 1][5]
        self.motor_stop(id_num=id_num)
        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        tx_data[0] = 0x01
        # Need to send using extended frame (data frame)
        self._send_can(id_num=id_num, 
                       cmd_mode=6, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        self._reply_state(id_num=id_num)
        if mode_status == 2:
            self.motor_enable(id_num=id_num)

    def set_id(self, 
               id_num=127, 
               new_id=1):
        '''Set motor ID number (saved after power off).

        Args:
            id_num: The ID number of the motor that needs to be reset. 
                    If you don't know the current motor number, 
                    you can broadcast with 0, but at this time 
                    only one motor can be connected to the bus, 
                    otherwise multiple motors will be set to the same number.
            new_id: Preset ID

        Returns:
            True: Setting successful
            False: Setting failed
        '''

        # Changing the ID number must be done in motor mode
        self.motor_stop(id_num=id_num)
        time.sleep(0.1)
        self.get_id(id_num=id_num)
        master_id = 0
        cmd_data = [0]*2
        cmd_data[1] = new_id & 0xFF
        cmd_data[0] = master_id & 0xFF
        if len(MCU_ID) == 8:
            tx_data = MCU_ID
            # Need to send using extended frame (data frame)
            self._send_can(id_num=id_num, 
                           cmd_mode=7, 
                           cmd_data=cmd_data, 
                           data=tx_data, 
                           rtr=0)
            time.sleep(0.1)
            self._reply_state(id_num=id_num)
            return True
        else:
            print("Set ID to " + str(new_id) + " failed!")
            return False

    def clear_error(self, 
                    id_num=127):
        '''Clear error function
        Once any error occurs during the motor operation, 
        if you want to restore the normal control mode, 
        you need to use clear_error to clear the error.

        Args:
            id_num: The ID number of the motor to clear the error flag

        Returns:
            None
        '''
        
        self.ERROR_FLAG = 'Status normal'
        master_id = 0
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        tx_data[0] = 0x01
        # Need to send using extended frame (data frame)
        self._send_can(id_num=id_num, 
                       cmd_mode=4, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        self._reply_state(id_num=id_num)

    def restore_config(self, 
                    id_num=127):
        '''Restore motor parameters to factory settings function.
        To restore all configuration parameters to the factory settings, 
        call this function.
        By default, after restoring to factory settings, CAN_ID will be 
        restored to 127, for convenience, this function automatically 
        changes it back to the original id_num.

        Args:
            id_num: The ID number of the motor to be modified

        Returns:
            None
        '''

        self.motor_stop(id_num=id_num)
        master_id = 0
        cmd_data = [0]*2
        cmd_data[1] = 0x03
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8
        for i in range(8):
            tx_data[i] = 0x00
        # Need to send using extended frame (data frame)
        self._send_can(id_num=id_num, 
                       cmd_mode=8, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        print("Restoring factory settings... Please wait for 3 seconds...")
        time.sleep(3.0)
        self.set_id(127, 
                    id_num)
        print("Successfully restored to factory settings!")

    def get_id(self, 
               id_num=127):
        '''Get motor ID number function

        Args:
            id_num: The ID number of the motor to be read

        Returns:
            id_num: Motor ID number
        '''

        global MCU_ID
        master_id = 0xFD
        cmd_data = [0]*2
        cmd_data[0] = master_id & 0xFF
        tx_data = [0]*8

        self._send_can(id_num=id_num, 
                       cmd_mode=0, 
                       cmd_data=cmd_data, 
                       data=tx_data, 
                       rtr=0)
        data = self._receive_can()
        if READ_FLAG == 1 and data[1] == 0:
            MCU_ID = data[5:]
            return id_num

    def get_posvel(self, 
                   id_num=127):
        '''Read the current position and speed of the motor,
        the units are degrees (°) and revolutions per minute (r/min) respectively.

        Args:
            id_num: The motor number to be read. 
                    If you don't know the current motor number, 
                    you can broadcast with 0, but at this time only one motor 
                    can be connected to the bus, otherwise it will report an error.

        Returns:
            [pos, vel]: Position and speed list

        Raises:
            "!!!ERROR IN GET STATE"
        '''

        pos_vel = [0, 0]
        try:
            # Call the master_ID write interface to get real-time position and speed 
            # through the motor response feedback frame
            self._write_prop(id_num=id_num, 
                             index=0x7018, 
                             value=27, 
                             data_type='f')
            if self.READ_FLAG == 1 and id_num != 0:
                pos_vel[0] = round(self.motor_state[id_num - 1][0], 1)
                pos_vel[1] = round(self.motor_state[id_num - 1][1], 1)
                return pos_vel[:]
            else:
                return
        except Exception as e:
            print("!!!ERROR IN GET STATE:", e)
            return False

    def get_volcur(self, 
                   id_num=127):
        '''Read the current voltage and current of the motor,
        the units are volts (V) and amperes (A) respectively.

        Args:
            id_num: The motor number to be read. 
                    If you don't know the current motor number, 
                    you can broadcast with 0, but at this time only one motor 
                    can be connected to the bus, otherwise it will report an error.

        Returns:
            [vol, cur]: Voltage and current list

        Raises:
            "!!!ERROR IN GET VOLTAGE AND CURRENT"
        '''
        global READ_FLAG
        vol_cur = [0, 0]
        try:
            vol_cur[0] = self._read_prop(id_num=id_num, 
                                         index=0x302b, 
                                         data_type='f')
            vol_cur[1] = self._read_prop(id_num=id_num, 
                                         index=0x301e, 
                                         data_type='f')
            if READ_FLAG == 1:
                vol_cur[0] = round(vol_cur[0], 1)
                vol_cur[1] = round(vol_cur[1], 2)
                return vol_cur
            else:
                return
        except Exception as e:
            print("!!!ERROR IN GET VOLTAGE AND CURRENT:", e)
            return False
