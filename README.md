# pyCyberGear: Control the CyberGear motor with Python

## Description
This is a Python-based tool that can be used to control the CyberGear motor by Xiaomi. It is designed to be easy to use for both beginners and experts.

## Installation
To install pyCyberGear, you can use the following command:
```bash
pip install pycybergear
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
cybergear.impedance_control(id_num='{MOTOR_ID}', pos='{ANGLE}', vel='{SPEED}', tff='{FEEDFORWARD_TORQUE}', kp='{PROPORTIONAL_GAIN}', kd='{DERIVATIVE_GAIN}')
```

- 'set_pos': Control the specified motor to rotate to the specified angle at the specified speed.
```python
cybergear.set_angle(id_num='{MOTOR_ID}', angle='{ANGLE}', speed='{SPEED}')
```

- 'set_speed': Control the specified motor to continuously rotate at the specified speed.
```python
cybergear.set_speed(id_num='{MOTOR_ID}', speed='{SPEED}')
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


## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
