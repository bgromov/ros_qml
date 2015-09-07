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

# TODO: Is there a way to determine self package name?
THIS_PACKAGE = 'ros_qml'

def sigint_handler(*args):
    sys.stderr.write('\r')
    QApplication.quit()

def ros_qml_main():
    try:
        rospy.init_node('ros_qml', sys.argv)
        my_argv = rospy.myargv(sys.argv)
        
        signal.signal(signal.SIGINT, sigint_handler)

        app = QApplication(my_argv)

        engine = QQmlEngine()
        engine.quit.connect(app.quit)

        plugins_dir = QLibraryInfo.location(QLibraryInfo.PluginsPath)

        # ## Add QML extension modules and plugins to the path, including ourselves
        plugins_paths = rospack.get_manifest(THIS_PACKAGE).get_export(THIS_PACKAGE, 'plugins')
        for idx, p in enumerate(plugins_paths):
            # If a relative path provided, treat it as relative to Qt plugins dir
            if not os.path.isabs(p):
                plugins_paths[idx] = plugins_dir + '/' + p

        qml_paths = rospack.get_manifest(THIS_PACKAGE).get_export(THIS_PACKAGE, 'imports')
        deps = rospack.get_depends_on(THIS_PACKAGE)
        for d in deps:
            pp = rospack.get_manifest(d).get_export(THIS_PACKAGE, 'plugins')
            for idx, p in enumerate(pp):
                # If a relative path provided, treat it as relative to Qt plugins dir
                if not os.path.isabs(p):
                    pp[idx] = plugins_dir + '/' + p

            plugins_paths += pp

            qp = rospack.get_manifest(d).get_export(THIS_PACKAGE, 'imports')
            qml_paths += qp

        for p in plugins_paths:
            engine.addPluginPath(p)

        for p in qml_paths:
            engine.addImportPath(p)

        qml_sys_path = QLibraryInfo.location(QLibraryInfo.ImportsPath)
        engine.addImportPath(qml_sys_path)

        # Somehow we need to set the path both with QQmlEngine and with environment,
        # commenting any of the two will lead to 'module not installed' error
        os.environ['QML2_IMPORT_PATH'] = ':'.join(qml_paths) + ':' + qml_sys_path

        comp = QQmlComponent(engine)
        
        if len(my_argv) > 1 and my_argv[1]:
            qml_url = my_argv[1]
            comp.loadUrl(QUrl(qml_url))
        elif rospy.has_param('qml_url'):
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
