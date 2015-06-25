import QtQuick 2.2
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.0

import Ros 1.0

ApplicationWindow {
    id: applicationWindow1
    visible: true
    property int margin: 11
    width: 640
    height: 480
    title: "ROS QML"

    Ros {
        id: ros
        Subscriber {
            id: sub1
            topic: Topic {
                name: "/my_node"
                type: "std_msgs/Header"
            }
            onMessage: {
                radioButton1.text = msg.stamp.nsecs
            }
        }
        Subscriber {
            id: joy_sub
            topic: Topic {
                name: "/joy"
                type: "sensor_msgs/Joy"
            }
            onMessage: {
                joyX.text = msg.axes[0].toFixed(3)
                joyY.text = msg.axes[1].toFixed(3)
                joyZ.text = msg.axes[2].toFixed(3)
            }
        }
        Publisher {
            id: pub1
            queue_size: 10
            topic: Topic {
                name: "/test_event"
                type: "std_msgs/Header"
            }
        }
        Publisher {
            id: pub2
            queue_size: 10
            topic: Topic {
                name: "/test_event2"
                type: "std_msgs/Empty"
            }
        }
    }

    menuBar: MenuBar {
        Menu {
            title: "&File"
            MenuItem {
                text: "E&xit"
                shortcut: StandardKey.Quit
                onTriggered: Qt.quit()
            }
        }
    }

    ColumnLayout {
        id: columnLayout1
        width: 200
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 0
        anchors.top: parent.top
        anchors.topMargin: 0
        anchors.right: parent.right
        anchors.rightMargin: 0

        Button {
            text: "Send"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            onClicked: {
                var now = ros.now()
                ros.logdebug('Sample debug message')
                ros.loginfo('Sample info message with user data: ' + JSON.stringify(now))
                ros.logwarn('Sample warn message')
                ros.logerr('Sample error message')
                ros.logfatal('Sample fatal message')
                var msg = {
                    seq: 0,
                    stamp: now,
                    frame_id: 'test_frame',
                }
                pub1.publish(msg)
            }
        }

        RadioButton {
            anchors.verticalCenterOffset: 30
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenterOffset: 0
            anchors.horizontalCenter: parent.horizontalCenter

            id: radioButton1
            text: qsTr("Radio Button")
        }
    }

    TextField {
        id: joyX
        x: 69
        y: 47
        placeholderText: "Joystick X"
        readOnly: true
    }

    TextField {
        id: joyY
        x: 69
        y: 78
        readOnly: true
        placeholderText: "Joystick Y"
    }

    Label {
        id: label1
        x: 53
        y: 51
        text: "X"
    }

    Label {
        id: label2
        x: 53
        y: 82
        text: "Y"
    }

    TextField {
        id: joyZ
        x: 69
        y: 109
        readOnly: true
        placeholderText: "Joystick Z"
    }

    Label {
        id: label3
        x: 53
        y: 113
        text: "Z"
    }
}
