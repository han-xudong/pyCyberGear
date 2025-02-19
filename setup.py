# -*- coding: utf-8 -*-
from setuptools import setup, find_packages

setup(
    name='pycybergear',
    version='0.1.0',
    description='Python library for controlling CyberGear motors',
    author='Xudong Han',
    url='https://github.com/han-xudong/pyCyberGear',
    packages=find_packages(),
    install_requires=['pyserial>=3.5', 'numpy>=2.0.1'],
    python_requires='>=3.6',
    keywords=['CyberGear', 'robotics', 'motor'],
    license='MIT',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
)