# Muto Agent

## Build
After you checkou the repository, ource your ROS environment and make sure the additional dependencies such as the eclipse paho mqtt client library is installed.

```bash
source /opt/ro/noetic/setup.bash
pip install paho-mqtt celery requests
```

## muto Twins

muto agent requires network connectivity to the muto Twins (ditto) and MQTT servers. The address for the muto sandbox is: 
* The twin servers and API: http://sandbox.composiv.ai
* The mqtt server: mqtt://sandbox.composiv.ai:1883
## Running agent

The muto agent must be launched and the target device and the devices should have access to all the packages required to launch the stacks.  Fo these purposes you can either choose to run a containerizedf version of muto or launch it direct from the devices after all the packages (such as the muto learning modules) are build and installed on the AV.

```bash
source devel/setup.bash
roslaunch muto_agent agent.launch
```

where agent.launch is:

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="muto_agent" name="muto_agent" type="muto_agent.py">
      <rosparam  command="load" file="$(find muto_agent)/params.yaml" />
  </node>
</launch>
```

and params.yaml is:

```yaml

muto:
  nav_topic: /nav
  type: simulator
  twin_url: "http://ditto:ditto@sandbox.composiv.ai"
  mqtt:
    host: sandbox.composiv.ai
    port: 1883
    keep_alive: 60
    user: none
    password: none
  thing:
    namespace: ai.composiv.sandbox.f1tenth
    anonymous: False  # Use this for automatically generated id (uuid based)
    #   if anonymous is True or anynoymous param is missing, name/id will be auto generated
    name: simulator-monster-01
```

## Twins
There are two definition types for thins on the wtin server:
* Stack - The model os the muto adaptive software stack that can be running on an AV
    -  Thing definition id for stack is: ai.composiv.sandbox.f1tenth:Stack:1.0.0
* Vehicle - The device twin for the AV, where parameters, telemetry data and stack associations are managed
    -  Thing definition id for vehicle is: ai.composiv.sandbox.f1tenth:TestCar:1.0.0


Examples of these definitions can be sampled by sending http rest requests to the sandbox server
http://sandbox.composiv.ai.  Muto Twin Server

```bash
curl 'http://sandbox.composiv.ai/api/2/search/things?filter=eq(definition,"ai.composiv.sandbox.f1tenth:Stack:1.0.0")'
```
returns:
```json
{
    "name": "Muto Learning Simulator with Gap Follwer",
    "context": "eteration_office",
    "stackId": "ai.composiv.sandbox.f1tenth:composir_simulator_gf.launch",
    "stack": [
        {
            "thingId": "ai.composiv.sandbox.f1tenth:composiv_simulator.launch"
        }
    ],
    "node": [ 
        {
            "name": "cass_gap_follower",
            "pkg": "cass_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              { "from": "$(find cass_gap_follower)/params.yaml" }
            ]
        }
    ]
}
```

This is a stack with a single node, "cass_gap_follower".  However, it includes another stack (with many other nodes and parameters) that it requires with a stackId reference ai.composiv.sandbox.f1tenth:muto_simulator.launch.  The elements of the stack model resembles a [ROS launch XML](https://wiki.ros.org/roslaunch/XML), so it should be fairly straightforward to understand if you have experience with it.

## Managing Stacks and Vehicles

New stack can be easily stored on the sandbox server using the things API (put). See the ditto documentation for many examples [https://www.eclipse.org/ditto/intro-overview.html].  For example, to add a new stack to the repository we can use the thing PUT api as follows:

```bash
$ curl -X PUT -H "Content-Type: application/json" -d ' 
{ 
    "name": "Muto Learning Simulator with Gap Follower", 
    "context": "eteration_office",
    "stackId": "ai.composiv.sandbox.f1tenth:composir_simulator_gf.launch", 
    "stack": [
        {
            "thingId": "ai.composiv.sandbox.f1tenth:composiv_simulator.launch"
        }
    ],
    "node": [ 
        {
            "name": "cass_gap_follower",
            "pkg": "cass_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              { "from": "$(find cass_gap_follower)/params.yaml" }
            ]
        }
    ]
}
' http://sandbox.composiv.ai/api/2/things/ai.composiv.sandbox.f1tenth:composiv_simulator_gf.launch

```

## Managing Stacks and Vehicles
We can use the TWINS to directly communicating commands to the vehicle itself. The twin server supports special mqtt channels for these purposes called **twin** and **live** channels. For example the following command can be published to the sandbox MQTT server to activate a stack on a car.  Each vehicle has its dedicated **twin** and **live** channels:

```yaml
topic: ai.composiv.sandbox.f1tenth:simulator-monster-01/stack/commands/active
```
```yaml
payload: {
    "name": "Muto Learning Simulator with Gap Follwer",
    "context": "eteration_office",
    "stackId": "ai.composiv.sandbox.f1tenth:composiv_simulator_gf.launch",
    "stack": [
        {
            "thingId": "ai.composiv.sandbox.f1tenth:composiv_simulator.launch"
        }
    ],
    "node": [ 
        {
            "name": "cass_gap_follower",
            "pkg": "cass_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              { "from": "$(find cass_gap_follower)/params.yaml" }
            ]
        }
    ]
}
```

You can use any open-source mqtt client to issue these commands and monitor various muto twin messages [MQTTX](https://mqttx.app/). Another option is the [mosquitto_pub](https://mosquitto.org/man/mosquitto_pub-1.html), which is a simple MQTT version 5/3.1.1 client that will publish a single message on a topic and exit.  You can publish the message described above using the commandline:

```bash
  mosquitto_pub -d -h sandbox.composiv.ai -p 1883  -t "ai.composiv.sandbox.f1tenth:simulator-monster-01/stack/commands/active" -m '{"name":"Muto Learning Simulator with Gap Follwer","context":"eteration_office","stackId":"ai.composiv.sandbox.f1tenth:muto_simulator_gf.launch","stack":[{"thingId":"ai.composiv.sandbox.f1tenth:composiv_simulator.launch"}],"node":[{"name":"cass_gap_follower","pkg":"cass_gap_follower","exec":"cass_gap_follower","param":[{"from":"$(find cass_gap_follower)/params.yaml"}]}]}'

```
