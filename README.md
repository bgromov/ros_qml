# ros_qml
The ros_qml package provides Qt 5.4 QML plugin for integration with ROS.

The plugin is written in Python and thus depends on PyQt 5.4.

To install PyQt with Python 2.7 bindings configure and build it with the following parameters (actual paths may differ):

```
$ python configure.py --sip-incdir=/usr/include/python2.7/ --qmake=/opt/qt54/bin/qmake
$ make
$ sudo make install 
```

## Nodes

 * `ros_qml` executes QML-file from location specified with `qml_url` parameter on ROS parameter server. The root component of executed QML-file has to be a QML window, e.g. `ApplicationWindow` (see sample code).

Run sample code with:
```
$ rosparam set /qml_url `rospack find ros_qml`/examples/main.qml
$ rosrun ros_qml ros_qml
```
or
```
$ rosparam set /qml_url `rospack find ros_qml`/examples/main.qml
$ roslaunch ros_qml ros_qml.launch
```

The launch file additionally defines `LIBOVERLAY_SCROLLBAR` environment variable and set it to 0 to avoid QT GTK+ bug on Ubuntu. If you do not have any scrollbars in your QML graphical interface then the variable can be omitted.

## Scripts

 * `qml_env.sh` &mdash; used internally to expose user-defined QML plugin dirs via Qt environment variables. That is particularly useful when QML files are edited with Qt Creator.

 * `qtcreator.sh` &mdash; bootstraps Qt Creator with all the necessary environment variables provided by `qml_env.sh`.

    ```
    $ rosrun ros_qml qtcreator.sh main.qml
    ```

To add your own QML plugins (either written in Python or C++) use `<ros_qml>` tag in export section of your `package.xml`, e.g.:

```xml
  <export>
    <ros_qml plugins="${prefix}/my_qml_plugins_dir" />
  </export>
```

If a relative path supplied to `plugins` attribute, it will be treated as relative to Qt plugins directory, e.g.:

```xml
  <export>
    <ros_qml plugins="PyQt5" /> <!-- This may resolve to /usr/lib/x86_64-linux-gnu/qt5/plugins/PyQt5 --/>
  </export>
```

To add your own extension modules (*.qml files) use `imports` attribute.

```xml
  <export>
    <ros_qml imports="${prefix}/my_qml_extensions_dir" plugins="PyQt5" />
  </export>
```

Note, however, that a path provided with `imports` attribute is always treated as absolute.

Then `qml_env.sh` script will take care of imports path to be exposed to Qt Creator.

## QML Plugin API

There are four components provided by the plugin:

 * `Ros` &mdash; top-level container for other components. It holds the reference to the actual ROS node and provides logging and time facilities.
 * `Subscriber` &mdash; subscribes to ROS topic and handles incomming messages. A topic to be used is defined with `Topic` component (see below).
 * `Publisher` &mdash; provides function to publish to ROS topic. Similarly to `Subscriber` uses `Topic` component.
 * `Topic` &mdash; just a container for topic specific properties, i.e. `name` and `type`.

See [examples/main.qml](examples/main.qml).

### Ros

```qml
Ros {
  /* Allows access to logging and time facilities */
  id: my_ros
  
  /* Array of publishers and subscribers.
   * That is a default container, so the name can be omitted (see below)
   */
  objects: [
    Subscriber {
      ...
    }
    Publisher {
      ...
    }
  ]
  
  /* The publishers and subscribers defined like this
   * will be automatically added to the 'objects' container
   */
  Publisher {
    ...
  }
}

Button {
  text: "Log message to ROS"
  onClicked: {
    var now = my_ros.now() // remember time stamp
    
    /* Logging functions correspond to those defined in rospy */
    my_ros.logdebug('Sample debug message')
    my_ros.loginfo('Sample info message with user data: ' + JSON.stringify(now))
    my_ros.logwarn('Sample warn message')
    my_ros.logerr('Sample error message')
    my_ros.logfatal('Sample fatal message')
  }
}
```

### Subscriber

```qml
Subscriber {
  /* Not used at the moment */
  id: sub1
  
  /* Corresponds to ROS subscriber queue_size parameter */
  queue_size: 10
  
  /* Holds topic name and type (see below) */
  topic: Topic {
    ...
  }
  
  /* Handles 'message' signal emitted by the plugin backend. 
   * Passes 'msg' as sole parameter, which is a JavaScript object
   */
  onMessage: {
    // QML JavaScript code goes here
  }
}
```

### Publisher

```qml
Publisher {
  /* Allows access to 'publish()' function */
  id: pub1
  
  /* Corresponds to ROS publisher queue_size parameter */
  queue_size: 10
  
  /* Holds topic name and type (see below) */
  topic: Topic {
    ...
  }
}
...
Button {
  text: "Publish"
  onClicked: {
    var my_msg = {
      field1: 'value1',
      field2: 'value2',
      ...
    }
    ...
    // call publish() method of Publisher component
    pub1.publish(my_msg)
  }
}
```

### Topic

```qml
Topic {
  name: "/my_qml_topic"
  type: "std_msgs/Header"
}
```
