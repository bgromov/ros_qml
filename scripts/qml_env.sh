#!/bin/bash

QML_DIRS=`rospack plugins --attrib=plugins ros_qml | cut -d' ' -f2- | tr '\r\n' ':'`

## Add paths to the end of existing vars
QML_IMPORT_PATH="$QML_IMPORT_PATH:$QML_DIRS"
QML2_IMPORT_PATH="$QML2_IMPORT_PATH:$QML_DIRS"

## Strip colons
QML_IMPORT_PATH=${QML_IMPORT_PATH%%:}
QML_IMPORT_PATH=${QML_IMPORT_PATH##:}
QML2_IMPORT_PATH=${QML2_IMPORT_PATH%%:}
QML2_IMPORT_PATH=${QML2_IMPORT_PATH##:}

export QML_IMPORT_PATH
export QML2_IMPORT_PATH
