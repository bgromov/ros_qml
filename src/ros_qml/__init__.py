#!/usr/bin/env python
import os
import sys
import signal
import subprocess

from OpenGL import GL  # work around for 'shader compilation failed' error
from PyQt5.QtCore import QUrl, QTimer, QLibraryInfo
from PyQt5.QtWidgets import QApplication
from PyQt5.QtQml import QQmlEngine, QQmlComponent, QQmlApplicationEngine

import rospy
import rospkg
rospack = rospkg.RosPack()

def sigint_handler(*args):
    sys.stderr.write('\r')
    QApplication.quit()

def ros_qml_main():
    try:
        rospy.init_node('ros_qml')
        
        signal.signal(signal.SIGINT, sigint_handler)

        app = QApplication(sys.argv)

        engine = QQmlEngine()
        engine.quit.connect(app.quit)

        plugins_dir = QLibraryInfo.location(QLibraryInfo.PluginsPath)
        engine.addPluginPath(plugins_dir + '/PyQt5')

        # ## Add ROS QML extension to the path
        # Somehow we need to set the path both with QQmlEngine and with environment,
        # commenting any of two will lead to 'module not installed' error
        engine.addImportPath(rospack.get_path('ros_qml') + '/plugins')
        os.environ['QML2_IMPORT_PATH'] = rospack.get_path('ros_qml') + '/plugins'

        comp = QQmlComponent(engine)
        
        if rospy.has_param('qml_url'):
            qml_url = rospy.get_param('qml_url')
            comp.loadUrl(QUrl(qml_url))
        elif rospy.has_param('qml_description'):  # experimental
            qml_description = rospy.get_param('qml_description')
            # FIXME that hangs for unknown reason
            comp.setData(QByteArray(qml_description), QUrl())
        else:
            rospy.logfatal('Neither /qml_url nor /qml_description (experimental) parameter is present')
            sys.exit(1)
        
        if not comp.isReady():
            sys.stderr.write(comp.errorString())
            sys.exit(1)
        
        win = comp.create()
        if not win:
            rospy.logfatal('Your root item has to be a Window')
            sys.exit(1)
        
        engine.setIncubationController(win.incubationController())
        if win:
            win.show()
        
        # Poll Python interpreter every 500ms
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        sys.exit(app.exec_())

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
