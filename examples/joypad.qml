import QtQuick 2.0
import QtQuick.Controls 1.0

import Ros 1.0

ApplicationWindow {
    id: appWindow
    visible: true
    width: 400
    height: 400

    Ros {
        id: ros

        Publisher {
            id: pubJoy
            queue_size: 10
            topic: Topic {
                name: "/twist"
                type: "geometry_msgs/TwistStamped"
            }
        }
    }

    Item {
        id: joyPad
        anchors.centerIn: parent
        width: 250
        height: 250

        Image {
            id: joyBase
            anchors.fill: parent
            source: "images/joystick_background.png"

            Image {
                id: joyThumb
                source: "images/joystick_thumb.png"
                x: cX
                y: cY

                readonly property int cX: (joyBase.width - width) / 2
                readonly property int cY: (joyBase.height - height) / 2

                property var twist: {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}

                function publishTwist() {
                    var header = {seq: 0, stamp: ros.now(), frame_id: ''}

                    twist.linear.x = (joyThumb.x - joyThumb.cX) * scaleX.value
                    twist.angular.x = (joyThumb.y - joyThumb.cY) * scaleY.value

                    pubJoy.publish({header: header, twist: joyThumb.twist})
                }

                onXChanged: {
                    if (mouseArea.drag.active) {
                        publishTwist()
                    }
                }

                onYChanged: {
                    if (mouseArea.drag.active) {
                        publishTwist()
                    }
                }

                MouseArea {
                    id: mouseArea
                    anchors.fill: parent
                    drag.target: joyThumb
                    drag.minimumX: 0
                    drag.minimumY: 0
                    drag.maximumX: joyBase.width - joyThumb.width
                    drag.maximumY: joyBase.height - joyThumb.height

                    onReleased: {
                        drag.target.x = drag.target.cX
                        drag.target.y = drag.target.cY
                    }
                }
            }
        }
    }

    Slider {
        id: scaleX
        tickmarksEnabled: true
        anchors.left: joyPad.left
        anchors.top: joyPad.bottom
        anchors.topMargin: 5
        anchors.right: joyPad.right
    }

    Slider {
        id: scaleY
        tickmarksEnabled: true
        anchors.top: joyPad.top
        anchors.right: joyPad.left
        anchors.rightMargin: 5
        anchors.bottom: joyPad.bottom
        orientation: Qt.Vertical
    }

    Component.onCompleted: {
        scaleX.value = 1.0
        scaleY.value = 1.0
    }
}
