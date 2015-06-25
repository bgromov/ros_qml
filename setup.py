#!/usr/bin/env python

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    packages = ['ros_qml'],
    package_dir = {'': 'src'},
    scripts = ['scripts/ros_qml'],
    requires = ['roslib', 'rospkg', 'rospy_message_converter']
)

setup(**d)
