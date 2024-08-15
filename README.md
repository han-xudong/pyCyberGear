# pyCyberGear: Control the CyberGear motor with Python

## Description
PyCyberGear is a Python library that allows you to control the CyberGear motor with Python. It provides a simple and easy-to-use interface to control the motor. The library suppprts:
- Enable and stop the motor
- Set motion mode: Impedance Control Mode, Position Mode, Speed Mode, Torque Mode
- Control the motor with the four modes
- Set zero position and ID
- Clear error and restore configuration
- Get the current position, velocity, voltage, and current

## Installation
The `pyCyberGear` package supports Python 3.9 through Python 3.12 on Windows and Linux.

Install the latest release with:
```bash
git clone https://github.com/han-xudong/pyCyberGear.git
cd pyCyberGear
pip install pip -U
pip install -e .
```

## Quick Start
Here is an example of how to init pyCyberGear:
```python
from pycybergear import CyberGear

cybergear = CyberGear(com_port="{YOUR_COM_PORT}", baud_rate=115200)
```

Here are some of the functions that you can use with pyCyberGear:
- 'motor_enable': Enable the motor
```python
cybergear.motor_enable(id_num='{MOTOR_ID}')
```

- 'motor_stop': Stop the motor.
```python
cybergear.motor_estop(id_num='{MOTOR_ID}')
```

- 'set_mode': Set motion mode: 0, Impedance Control Mode; 1, Position Mode; 2, Speed Mode; 3, Torque Mode.
```python
cybergear.set_mode(id_num='{MOTOR_ID}', mode=0)
```

- 'impedance_control': Control the specified motor with impedance.
```python
cybergear.impedance_control(id_num='{MOTOR_ID}', pos='{ANGLE}', vel='{VELOCITY}', tff='{FEEDFORWARD_TORQUE}', kp='{PROPORTIONAL_GAIN}', kd='{DERIVATIVE_GAIN}')
```

- 'set_pos': Control the specified motor to rotate to the specified angle at the specified speed.
```python
cybergear.set_pos(id_num='{MOTOR_ID}', pos='{ANGLE}', vel='{VELOCITY}')
```

- 'set_vel': Control the specified motor to rotate at the specified speed.
```python
cybergear.set_vel(id_num='{MOTOR_ID}', vel='{VELOCITY}')
```

- 'set_torque': Control the specified motor to rotate to output the specified torque (Nm).
```python
cybergear.set_torque(id_num='{MOTOR_ID}', torque='{TORQUE}')
```

- 'set_zero': Set the current position of the motor as the zero position.
```python
cybergear.set_zero(id_num='{MOTOR_ID}')
```

- 'set_id': Set the ID of the motor.
```python
cybergear.set_id(id_num='{MOTOR_ID}', new_id='{NEW_MOTOR_ID}')
```

- 'clear_error': Clear the error of the motor.
```python
cybergear.clear_error(id_num='{MOTOR_ID}')
```

- 'restore_config': Restore motor parameters to factory settings.
```python
cybergear.restore_config(id_num='{MOTOR_ID}')
```

- 'get_posvel': Get the current position and velocity of the motor.
```python
cybergear.get_posvel(id_num='{MOTOR_ID}')
```

- 'get_volcur': Get the current voltage and current of the motor.
```python
cybergear.get_volcur(id_num='{MOTOR_ID}')
```

## Examples
Here are several examples in the 'examples' folder that you can refer to:


## License and Acknowledgements
PyCyberGear is licensed under the MIT License.

We sincerely thank the developers of the [DrEmpower Wiki](https://gitee.com/lyh458/drempower-wiki) for the core implementation of the CyberGear motor control library with their USB-to-CAN adapter.