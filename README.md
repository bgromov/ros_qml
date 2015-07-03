# ros_qml
The ros_qml package provides Qt 5.4 QML plugin for integration with ROS.

The plugin is written in Python and thus depends on PyQt 5.4.

## Usage

There are four components provided by the plugin:

 * `Ros` &mdash; top-level container for other components. It holds the reference to the actual ROS node and provides logging and time facilities.
 * `Subscriber` &mdash; subscribes to ROS topic and handles incomming messages. A topic to be used is defined with `Topic` component (see below).
 * `Publisher` &mdash; provides function to publish to ROS topic. Similarly to `Subscriber` uses `Topic` component.
 * `Topic` &mdash; just a container for topic specific properties, i.e. `name` and `type`.

See [examples/main.qml](examples/main.qml).

## API

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
  
  /* Correspondes to ROS subscriber queue_size parameter */
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
  
  /* Correspondes to ROS publisher queue_size parameter */
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
