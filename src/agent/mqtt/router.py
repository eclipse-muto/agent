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
import re
from muto_msgs.msg import MutoAction

class Router(object):

    def __init__(self, topic, publisher):
        self.publisher = publisher
        self.topic = topic
   
    def route(self,context, action, payload):
        print("Compose route:", action, payload)
        mesg = MutoAction()
        mesg.context = json.dumps(context)
        mesg.payload = payload
        expressions = re.findall('.*/stack/commands/(.*)', action)
        for method in expressions:
          mesg.method = method
          self.publisher.publish(mesg)
          return json.dumps({ "result": 'success', "command": action })
        expressions = re.findall('.*/agent/commands/(.*)', action)
        for method in expressions:
          mesg.method = method
          return action + "is not implemented"
        return "None"
