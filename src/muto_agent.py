#
#  Copyright (c) 2022 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#
#!/usr/bin/env python

import json
import uuid

import rospy
import paho.mqtt.client as mqtt
import paho.mqtt.packettypes as packettypes

import core.ditto.twin as twin
import core.model.edge_device as edge
import agent.mqtt.router as router


from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from muto_msgs.msg import MutoAction


class MutoAgent(object):
    """
    The class that handles muto
    """

    def __init__(self):

        self.muto = rospy.get_param("muto_agent/muto") 
        self.mqtt = rospy.get_param("muto_agent/muto/mqtt") 

        self.twin = twin.Twin(node='muto_agent',config=self.muto)

       # Subscribers
        rospy.Subscriber(rospy.get_param("muto_agent/muto/nav_topic"),AckermannDriveStamped, self.on_nav_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("muto_agent/muto/twin_topic"),String, self.on_twin_callback, queue_size=100)
        self.action_publisher = rospy.Publisher(rospy.get_param("muto_agent/muto/stack_topic"), MutoAction, queue_size=10)
        self.router = router.Router(self.twin.topic, self.action_publisher)


        self.mqtt_client = mqtt.Client(self.twin.uniqueName, reconnect_on_failure=True, protocol=mqtt.MQTTv5)
        self.mqtt_client.on_connect = lambda xx, userdata, flags, reason, properties: self.on_mqtt_connect(userdata, flags,reason, properties)
        self.mqtt_client.on_message = lambda xx, userdata, msg: self.on_mqtt_message(userdata, msg)


        self.edge_device = edge.EdgeDevice(self.twin, self.muto)
        self.edge_device.connect()

        try:
            self.mqtt_client.connect(self.mqtt['host'], self.mqtt['port'],  self.mqtt['keep_alive'])
        except Exception as error:
         print('An exception occurred connecting to mqtt: {}'.format(error))

        self.mqtt_client.loop_start()
        
    def cleanup(self) :
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def on_mqtt_connect(self, userdata, flags,reason,properties):
        print("Connected with result code "+str(reason))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        topic = self.twin.topic

        print("Subscribed to: ", topic +"/#")
        self.mqtt_client.subscribe(topic+"/#")

    def on_twin_callback(self, mqttmsg):
        topic = "muto/"+self.twin.thingId
        self.mqtt_client.publish(topic, mqttmsg.data, qos=0)


    def on_mqtt_message(self, userdata, msg):
        # The callback for when a PUBLISH message is received from the server.
        try:
            m_decode=str(msg.payload.decode("utf-8","ignore"))
            response_topic = None
            if hasattr(msg.properties, "ResponseTopic") :
                response_topic = msg.properties.ResponseTopic
            correlationData = None
            if hasattr(msg.properties, "CorrelationData") :
                correlationData = msg.properties.CorrelationData

            #payload = json.loads(m_decode)
            response  = self.router.route(self.twin.getContext(), msg.topic, m_decode)
            if not response_topic is None and not response is None:
                properties=mqtt.Properties(packettypes.PacketTypes.PUBLISH)
                properties.CorrelationData=correlationData
                print('Responding on response topic:', properties)
                #respond
                self.mqtt_client.publish(response_topic,response,properties=properties)

        except Exception as e:
            print("MQTT Message Received failed:", str(e))

    # Input data is AckermannDriveStamped message from nav topic
    # Publishes velocity and steering angle to twin channel
    def on_nav_callback(self, msg):
        if self.mqtt_client.is_connected:
            self.twin.publishTelemetry("/features/telemetry", 
            {"properties": { "velocity": msg.drive.speed, "steering_angle":  msg.drive.steering_angle }})



def main():
    rospy.init_node(name="muto_agent",anonymous=True)
    C = MutoAgent()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
