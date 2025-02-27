# pyCyberGear: Control the CyberGear motor with Python

## Description

PyCyberGear is a Python library that allows you to control the CyberGear motor with Python. It provides a simple and easy-to-use interface to control the motor. The library suppprts:

- Enable and stop the motor
- Set motion mode: Impedance Control Mode, Position Mode, Speed Mode, Torque Mode
- Control the motor with the four modes
- Set zero position and ID
- Clear error and restore configuration
- Get the current position, velocity, voltage, and current

## Hardware Requirements

To use pyCyberGear, you need the following hardware:

- [CyberGear motor](https://www.mi.com/cyber-gear)
- [DR USB-to-CAN adapter](https://item.taobao.com/item.htm?id=737373782475) or [YourCee USB-to-CAN adapter](https://item.taobao.com/item.htm?id=635000838271&skuId=4710513306912)
- 24V10A power supply
- Power board

## Installation

The `pyCyberGear` library supports Python 3.9 - 3.12, tested on Windows 11 and Ubuntu 22.04.

Install the latest release with:

```bash
git clone https://github.com/han-xudong/pyCyberGear.git
cd pyCyberGear
pip install .
```

## Quick Start

Here is an example of how to init pyCyberGear:

```python
from pycybergear import CyberGear

cybergear = CyberGear(com_port="{YOUR_COM_PORT}", baud_rate=115200, model={YOUR_USB-TO-CAN})
```

Two kinds of USB-to-CAN adaptors are supported, including DR's (model="DR") and YourCee's (model="CAN").

Here are some of the functions that you can use with pyCyberGear:

- `motor_enable`: Enable the motor

```python
cybergear.motor_enable(id_num='{MOTOR_ID}')
```

- `motor_stop`: Stop the motor.

```python
cybergear.motor_estop(id_num='{MOTOR_ID}')
```

- `set_mode`: Set motion mode: 0, Impedance Control Mode; 1, Position Mode; 2, Speed Mode; 3, Torque Mode.

```python
cybergear.set_mode(id_num='{MOTOR_ID}', mode=0)
```

- `impedance_control`: Control the specified motor with impedance.

```python
cybergear.impedance_control(id_num='{MOTOR_ID}', pos='{ANGLE}', vel='{VELOCITY}', tff='{FEEDFORWARD_TORQUE}', kp='{PROPORTIONAL_GAIN}', kd='{DERIVATIVE_GAIN}')
```

- 'set_pos`: Control the specified motor to rotate to the specified angle at the specified speed.

```python
cybergear.set_pos(id_num='{MOTOR_ID}', pos='{ANGLE}', vel='{VELOCITY}')
```

- `set_vel`: Control the specified motor to rotate at the specified speed.

```python
cybergear.set_vel(id_num='{MOTOR_ID}', vel='{VELOCITY}')
```

- `set_torque`: Control the specified motor to rotate to output the specified torque (Nm).

```python
cybergear.set_torque(id_num='{MOTOR_ID}', torque='{TORQUE}')
```

- `set_zero`: Set the current position of the motor as the zero position.

```python
cybergear.set_zero(id_num='{MOTOR_ID}')
```

- `set_id`: Set the ID of the motor.

```python
cybergear.set_id(id_num='{MOTOR_ID}', new_id='{NEW_MOTOR_ID}')
```

- `clear_error`: Clear the error of the motor.

```python
cybergear.clear_error(id_num='{MOTOR_ID}')
```

- `restore_config`: Restore motor parameters to factory settings.

```python
cybergear.restore_config(id_num='{MOTOR_ID}')
```

- `get_posvel`: Get the current position and velocity of the motor.

```python
cybergear.get_posvel(id_num='{MOTOR_ID}')
```

- `get_volcur`: Get the current voltage and current of the motor.

```python
cybergear.get_volcur(id_num='{MOTOR_ID}')
```

## Examples

Here are several examples in the 'examples' folder that you can refer to:

- `cybergear_position_mode`: Control the motor in position mode.

```bash
python examples/cybergear_position_mode.py
```

- `cybergear_speed_mode`: Control the motor in speed mode.

```bash
python examples/cybergear_speed_mode.py
```

- `cybergear_loop`: Control the motor for loop motion.

```bash
python examples/cybergear_loop.py
```

## License and Acknowledgement

PyCyberGear is licensed under the MIT License.

We sincerely thank the developers of the [DrEmpower Wiki](https://github.com/DrRobotTech/drempower-wiki) for the core implementation of the CyberGear motor control library with their USB-to-CAN adapter.
