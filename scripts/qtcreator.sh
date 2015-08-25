#!/bin/bash

source `rospack find ros_qml`/scripts/qml_env.sh

if [[ ! -z $@ ]]; then
    ## Assume the last argument is a filename
    last_arg="${!#}"
    ## All arguments except the last one
    other_args=${@:1:$(($#-1))}

    ## Normalize the filename
    qml_file_arg="$(cd "$(dirname "$last_arg")"; pwd)/$(basename "$last_arg")"
fi

## Following platform detection snipped adopted from ROS setup.sh
# detect if running on Darwin platform
_UNAME=`uname -s`
_IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
    _IS_DARWIN=1
fi
unset _UNAME

if [ $_IS_DARWIN -eq 0 ]; then
    qtcreator "$other_args" "$qml_file_arg"
else
    launchctl setenv QML_IMPORT_PATH $QML_IMPORT_PATH
    launchctl setenv QML2_IMPORT_PATH $QML2_IMPORT_PATH
    open -a "Qt Creator" --args "$other_args" "$qml_file_arg"
    launchctl unsetenv QML_IMPORT_PATH
    launchctl unsetenv QML2_IMPORT_PATH
fi
unset _IS_DARWIN
