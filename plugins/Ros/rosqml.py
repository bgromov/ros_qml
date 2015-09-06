import sys

try:
    sys.argv
except:
    setattr(sys, 'argv', [])
    setattr(sys, 'argc', 0)

from PyQt5.QtCore import pyqtProperty, pyqtSlot, pyqtSignal, QVariant
from PyQt5.QtCore import Q_CLASSINFO
from PyQt5.QtQml import QQmlListProperty, QQmlComponent, QQmlParserStatus
from PyQt5.QtQuick import QQuickItem

import rospy
import rospkg
import roslib
from rospy_message_converter import message_converter, json_message_converter

class Topic(QQuickItem, QQmlParserStatus):
    def __init__(self, parent = None):
        super(Topic, self).__init__(parent)
        self._name = None
        self._type = None

    @pyqtProperty(str)
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        self._name = str(name)

    @pyqtProperty(str)
    def type(self):
        return self._type

    @type.setter
    def type(self, type):
        self._type = str(type)

    def componentComplete(self):
        if not self.name:
            rospy.logfatal('QML property Topic.name is not set')
            sys.exit(1)
            
        if not self.type:
            rospy.logfatal('QML property Topic.type is not set')
            sys.exit(1)
        
class Subscriber(QQuickItem, QQmlParserStatus):
    on_message = pyqtSignal('QVariantMap', name = 'message', arguments = ['msg'])

    def __init__(self, parent = None):
        super(Subscriber, self).__init__(parent)
        self._topic = None
        self._queue_size = None
        self._sub = None
    
    @pyqtProperty(Topic)
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, topic):
        self._topic = topic
        
    @pyqtProperty(int)
    def queue_size(self):
        return self._queue_size

    @queue_size.setter
    def queue_size(self, queue_size):
        self._queue_size = queue_size

    def componentComplete(self):
        if not self.topic:
            rospy.logfatal('QML property Subscriber.topic is not set')
            sys.exit(1)
            
    def onMessage(self, data):
        self.on_message.emit(message_converter.convert_ros_message_to_dictionary(data))
            
    def subscribe(self):
        message_class = roslib.message.get_message_class(self.topic.type)
        self._sub = rospy.Subscriber(self.topic.name, message_class, self.onMessage, queue_size = self.queue_size)
    
class Publisher(QQuickItem, QQmlParserStatus):
    def __init__(self, parent = None):
        super(Publisher, self).__init__(parent)
        self._topic = None
        self._queue_size = None
        self._pub = None

    @pyqtProperty(Topic)
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, topic):
        self._topic = topic

    @pyqtProperty(int)
    def queue_size(self):
        return self._queue_size

    @queue_size.setter
    def queue_size(self, queue_size):
        self._queue_size = queue_size

    def componentComplete(self):
        if self._topic is None:
            rospy.logfatal('QML property Publisher.topic is not set')
            sys.exit(1)

        message_class = roslib.message.get_message_class(self.topic.type)
        self._pub = rospy.Publisher(self.topic.name, message_class, queue_size = self.queue_size)

    @pyqtSlot(str, name = 'publish')
    @pyqtSlot('QVariantMap', name = 'publish')
    def publish(self, msg):
        if isinstance(msg, str) or isinstance(msg, unicode):
            # Message is a JSON string
            self._pub.publish(json_message_converter.convert_json_to_ros_message(self.topic.type, msg))
        elif isinstance(msg, dict):
            # Message is Python dictionary
            self._pub.publish(message_converter.convert_dictionary_to_ros_message(self.topic.type, msg))

class Ros(QQuickItem, QQmlParserStatus):
    Q_CLASSINFO('DefaultProperty', 'objects')

    def __init__(self, parent = None):
        super(Ros, self).__init__(parent)

        self._publishers = []
        self._subscribers = []

    @pyqtProperty(QQmlListProperty)
    def objects(self):
        return QQmlListProperty(QQuickItem, self, append = self.append_object)

    @pyqtProperty(bool)
    def isInitialized(self):
        return rospy.core.is_initialized()

    def append_object(self, node, object):
        if isinstance(object, Subscriber):
            self._subscribers.append(object)
        elif isinstance(object, Publisher):
            self._publishers.append(object)
        
    def componentComplete(self):
        for s in self._subscribers:
            s.subscribe()
            rospy.logdebug('Subscribed to: ' + s.topic.name)

        for p in self._publishers:
            rospy.logdebug('Will publish to: ' + p.topic.name)

    @pyqtSlot(name = 'now', result = 'QVariantMap')
    def now(self):
        now = rospy.Time.now()
        return {'secs': now.secs, 'nsecs': now.nsecs}

    @pyqtSlot(str, name = 'logdebug')
    def logdebug(self, s):
        rospy.logdebug(s)

    @pyqtSlot(str, name = 'loginfo')
    def loginfo(self, s):
        rospy.loginfo(s)

    @pyqtSlot(str, name = 'logwarn')
    def logwarn(self, s):
        rospy.logwarn(s)

    @pyqtSlot(str, name = 'logerr')
    def logerr(self, s):
        rospy.logerr(s)

    @pyqtSlot(str, name = 'logfatal')
    def logfatal(self, s):
        rospy.logfatal(s)
