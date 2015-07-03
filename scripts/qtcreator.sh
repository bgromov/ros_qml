#!/bin/bash

source `rospack find ros_qml`/scripts/qml_env.sh

## Following platform detection snipped adopted from ROS setup.sh
# detect if running on Darwin platform
_UNAME=`uname -s`
_IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
    _IS_DARWIN=1
fi
unset _UNAME

if [ $_IS_DARWIN -eq 0 ]; then
    qtcreator $@
else
    launchctl setenv QML_IMPORT_PATH $QML_IMPORT_PATH
    launchctl setenv QML2_IMPORT_PATH $QML2_IMPORT_PATH
    open -a "Qt Creator" --args $@
    launchctl unsetenv QML_IMPORT_PATH
    launchctl unsetenv QML2_IMPORT_PATH
fi
unset _IS_DARWIN
