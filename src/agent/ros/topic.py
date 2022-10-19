import json
import socket
import rospy
import rosgraph
import rostopic

import agent.ros.echo as echo
import agent.ros.command as command

import muto_msgs.srv as muto_srv




class RosTopicCommands(object):

    def __init__(self, twin, mqtt_client):
        self.mqtt_client = mqtt_client
        self.twin = twin
        self.echo = {}
        self.rostopic_info = rospy.Service(
            "rostopic_info", muto_srv.CommandPlugin, self.handle_topic_info)
        self.rostopic_list = rospy.Service(
            "rostopic_list", muto_srv.CommandPlugin, self.handle_topic_list)
        self.rostopic_echo = rospy.Service(
            "rostopic_echo", muto_srv.CommandPlugin, self.handle_topic_echo)
    

    # rostopic_info service
    def handle_topic_info(self, req):
        payload = json.loads(req.input.payload)
        requested_topic = payload["topic"]
        topic_type, real_topic, msg_eval = rostopic.get_topic_type(requested_topic)
        status = { "status": "NOTFOUND"}
        if real_topic is not None:
            info = self.get_topic_info(requested_topic)
            msg = command.to_stdmsgs_string(json.dumps(info))
            return command.to_commandoutput(msg.data)
        msg = command.to_stdmsgs_string(json.dumps(status))
        return command.to_commandoutput(msg.data)

    #
    # Adapted from rospy
    #
    def get_topic_info(self, topic):
        import itertools
        def topic_type(t, topic_types):
            matches = [t_type for t_name, t_type in topic_types if t_name == t]
            if matches:
                return matches[0]
            return 'unknown type'

        master = rosgraph.Master('/rostopic')
        try:
            pubs, subs = rostopic.get_topic_list(master=master)
            # filter based on topic
            subs = [x for x in subs if x[0] == topic]
            pubs = [x for x in pubs if x[0] == topic]

            topic_types = rostopic._master_get_topic_types(master)

        except socket.error:
            raise rostopic.ROSTopicIOException("Unable to communicate with master!")

        if not pubs and not subs:
            raise rostopic.ROSTopicException("Unknown topic %s" % topic)

        info = {"topic": topic, "types": topic_type(topic, topic_types), "pubs": [], "subs": []}
        if pubs:
            for p in itertools.chain(*[nodes for topic, ttype, nodes in pubs]):
                info["pubs"].append({"publisher": p, "api": rostopic.get_api(master, p)})

        if subs:
            for p in itertools.chain(*[nodes for topic, ttype, nodes in subs]):
                info["subs"].append({"subscriber": p, "api": rostopic.get_api(master, p)})
        return info


    # rostopic list service
    def handle_topic_list(self, req):
        master = rosgraph.Master('/rostopic')
        pubs, subs = rostopic.get_topic_list(master=master)
        msg = command.to_stdmsgs_string(json.dumps({"pubs": pubs, "subs": subs}))
        return command.to_commandoutput(msg.data)

    # rostopic_echo service
    def handle_topic_echo(self, req):
        payload = json.loads(req.input.payload)
        topic = payload.get('topic')
        action = payload.get('action')
        status = { "status": "NOTFOUND", "topic": topic}
        try:
            if action == "reset":
                for e in self.echo:
                    current = self.echo.get(e)
                    if not current is None:
                        current.stop()
                status = { "status": "RESET", "topic": "all topics" }
                msg = command.to_stdmsgs_string(json.dumps(status))
                return command.to_commandoutput(msg.data)
            if not topic is None:
                current = self.echo.get(topic)
                if not current is None:
                    current.stop()
                    status = { "status": "STOPPED", "topic": topic }
                if action == "stop":
                    newTopicEcho = echo.TopicEcho(payload, self.twin, self.mqtt_client)
                    self.echo[topic] = newTopicEcho
                    newTopicEcho.register()
                    newTopicEcho.stop()
                    status = { "status": "STOPPED", "topic": topic }
                if action == "start":
                    newTopicEcho = echo.TopicEcho(payload, self.twin, self.mqtt_client)
                    self.echo[topic] = newTopicEcho
                    newTopicEcho.register()
                    newTopicEcho.start()
                    status = { "status": "STARTED", "topic": topic }
                if action == "delete":
                    newTopicEcho = echo.TopicEcho(payload, self.twin, self.mqtt_client)
                    newTopicEcho.delete()
                    if not current is None:
                        self.echo.pop(topic)
                    status = { "status": "DELETED", "topic": topic }
                if action == "register":
                    newTopicEcho = echo.TopicEcho(payload, self.twin, self.mqtt_client)
                    self.echo[topic] = newTopicEcho
                    newTopicEcho.register()
        except Exception as e:
            status = { "status": "EXCEPTION", "exception": "{}".format(e) }

        msg = command.to_stdmsgs_string(json.dumps(status))
        return command.to_commandoutput(msg.data)

