
import paho.mqtt.client as mqtt
import paho.mqtt.packettypes as packettypes
import time

import rospy
import rostopic

from std_msgs.msg import *
from muto_msgs.srv import *
from muto_msgs.msg import *
from ackermann_msgs.msg import *
from nav_msgs.msg import *

class TopicEcho(object):

    def __init__(self, manifest, mqtt_client):
        
        self.topic = manifest.get("topic")
        self.target = manifest.get("target")
        self.filter = manifest.get("filter")
        self.count = manifest.get("count")
        self.action =  manifest.get("action")
        self.mqtt_client = mqtt_client
        
        if not manifest.get("rate", "5000") is None:
            self.rate = float(manifest.get("rate", "5000"))
        
        self.lastpublish = time.monotonic_ns() * 0.001
        
        ttype, tname,_x = rostopic.get_topic_type(self.topic)
        self.topic_type =  eval(ttype.split("/")[1])
        self.topic = tname
                
    
    def subscribe(self):
        self.sub = rospy.Subscriber(self.topic,
                                    self.topic_type, 
                                    self.on_topic_callback,
                                    queue_size=1)

    def start(self):
        self.lastpublish = 0
        self.subscribe()
        
    def stop(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None

    def on_topic_callback(self, msg):
        if msg:
            current = time.monotonic_ns() * 0.001
            if current - self.lastpublish > self.rate:
                self.lastpublish = current
                data = str(msg)
                if not self.target is None and not data is None:
                    properties= mqtt.Properties(packettypes.PacketTypes.PUBLISH)
                    properties.CorrelationData= bytearray(self.target["correlation"],"utf8")
                    #respond
                    self.mqtt_client.publish(self.target["topic"],data,properties=properties)
                
