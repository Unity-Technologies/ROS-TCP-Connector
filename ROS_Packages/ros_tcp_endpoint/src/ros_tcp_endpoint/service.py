#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rospy
from rospy.service import ServiceException

from .communication import RosSender


class RosService(RosSender):
    """
    Class to send messages to a ROS service.
    """

    def __init__(self, service, service_class):
        """
        Args:
            service:        The service name in ROS
            service_class:  The service class in catkin workspace
        """
        RosSender.__init__(self)
        self.srv_class = service_class._request_class()
        self.srv = rospy.ServiceProxy(service, service_class)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's class, calls the service with the message, and returns
        the service's response.

        Args:
            data: The already serialized message_class data coming from outside of ROS

        Returns:
            service response
        """
        message = self.srv_class.deserialize(data)

        attempt = 0

        while attempt < 3:
            try:
                service_response = self.srv(message)
                return service_response
            except ServiceException:
                attempt += 1
                print("Service Exception raised. Attempt: {}".format(attempt))
            except Exception as e:
                print("Exception Raised: {}".format(e))

        return None

    def unregister(self):
        """

        Returns:

        """
        if not self.srv is None:
            self.srv.close()
