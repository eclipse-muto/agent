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

import rospy
import json
from rospy import ROSException
from threading import Thread
import agent.ros.command as command
from muto_msgs.msg import CommandOutput


class AsyncServiceProxyException(Exception):
    pass

class AsyncServiceProxy(object):
    def __init__(self, name, service_class, persistent=True):
        try:
            rospy.wait_for_service(service=name,timeout=15.0)
        except ROSException:
            raise AsyncServiceProxyException("Service {} is not available".format(name))
        self._service_proxy = rospy.ServiceProxy(name, service_class, persistent)
        self._thread = Thread(target=self._service_proxy, args=[])

    def __del__(self):
        # try to join the service thread - 
        # TODO: If there is an ongoing request interrupt, fail or queue?
        if self._thread.is_alive():
            self._thread.join(1.0)

    def __call__(self, request):
        if self._thread.is_alive():
            self._thread.join(0.01)
            if self._thread.is_alive():
                return False

        self._thread = Thread(target=self._service_proxy, args=[request])
        self._thread.start()
        status = { "status": "Started"}
        msg = command.to_stdmsgs_string(json.dumps(status))
        return command.to_commandoutput(msg.data)