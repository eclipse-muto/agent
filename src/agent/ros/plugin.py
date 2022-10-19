import json
import agent.ros.command as rcommand
import agent.ros.node as rnode
import agent.ros.topic as rtopic
import agent.ros.param as rparam


class CommandRouter(object):

    def __init__(self, commands):
        self.commands = commands

    def route(self, msg, payload):
        print("Command route:{}".format(msg.method))
        if self.commands.get(msg.method) is not None:
            return self.commands[msg.method].execute(msg, payload)


class RosCommandsPlugin(object):

    def __init__(self, manifest, twin, mqtt_client):
        self.manifest = manifest
        self.twin = twin
        self.sub = None
        self.topic_timer = None
        self.mqtt_client = mqtt_client
        self.echo = {}
        
        self.node_cmds = rnode.RosNodeCommands(mqtt_client)
        self.topic_cmds = rtopic.RosTopicCommands(twin, mqtt_client)
        self.param_cmds = rparam.RosParamCommands(mqtt_client)

        self.bootstrap()

    def bootstrap(self):
        if self.manifest:
            self.commands = {}
            for commandItem in self.manifest:
                command = rcommand.Command(commandItem)
                self.commands[commandItem["name"]] = command
            self.router = CommandRouter(self.commands)

    def on_command_callback(self, msg):
        if msg:
            payload = json.loads(msg.payload)
            return self.router.route(msg, payload)
