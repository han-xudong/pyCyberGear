# pyCyberGear: Control the CyberGear motor with Python

## Description
This is a Python-based tool that can be used to control the CyberGear motor by Xiaomi. It is designed to be easy to use and can be used by both beginners and experts.

## Installation
To install CyberGear with Python, you can use the following command:
```bash
pip install pycybergear
```

## Quick Start
Here is an example of how you can use CyberGear with Python:
```python
from pycybergear import CyberGear

cybergear = CyberGear(com_port="{YOUR_COM_PORT}", baud_rate=115200)

```