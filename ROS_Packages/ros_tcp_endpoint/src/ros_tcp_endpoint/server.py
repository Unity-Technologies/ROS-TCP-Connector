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

import sys
import json
import rospy
import socket
import logging
import threading
import importlib

from .tcp_sender import UnityTcpSender
from .client import ClientThread
from .subscriber import RosSubscriber
from .publisher import RosPublisher
from .service import RosService
from .unity_service import UnityService
from unity_interfaces.msg import RosUnitySysCommand
from unity_interfaces.srv import RosUnityTopicListResponse


class TcpServer:
    """
    Initializes ROS node and TCP server.
    """

    def __init__(self, node_name, buffer_size=1024, connections=2, tcp_ip="", tcp_port=-1):
        """
        Initializes ROS node and class variables.

        Args:
            node_name:               ROS node name for executing code
            buffer_size:             The read buffer size used when reading from a socket
            connections:             Max number of queued connections. See Python Socket documentation
        """
        if tcp_ip != "":
            self.tcp_ip = tcp_ip
        else:
            self.tcp_ip = rospy.get_param("/ROS_IP")

        if tcp_port != -1:
            self.tcp_port = tcp_port
        else:
            self.tcp_port = rospy.get_param("/ROS_TCP_PORT", 10000)

        self.unity_tcp_sender = UnityTcpSender()

        self.node_name = node_name
        self.source_destination_dict = {}
        self.buffer_size = buffer_size
        self.connections = connections
        self.syscommands = SysCommands(self)

    def start(self, source_destination_dict=None):
        if source_destination_dict is not None:
            self.source_destination_dict = source_destination_dict
        server_thread = threading.Thread(target=self.listen_loop)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()

    def listen_loop(self):
        """
        Creates and binds sockets using TCP variables then listens for incoming connections.
        For each new connection a client thread will be created to handle communication.
        """
        rospy.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.tcp_ip, self.tcp_port))

        while True:
            tcp_server.listen(self.connections)

            try:
                (conn, (ip, port)) = tcp_server.accept()
                ClientThread(conn, self, ip, port).start()
            except socket.timeout as err:
                logging.exception("ros_tcp_endpoint.TcpServer: socket timeout")

    def send_unity_error(self, error):
        self.unity_tcp_sender.send_unity_error(error)

    def send_unity_message(self, topic, message):
        self.unity_tcp_sender.send_unity_message(topic, message)

    def send_unity_service(self, topic, service_class, request):
        return self.unity_tcp_sender.send_unity_service(topic, service_class, request)

    def send_unity_service_response(self, srv_id, data):
        self.unity_tcp_sender.send_unity_service_response(srv_id, data)

    def topic_list(self, data):
        return RosUnityTopicListResponse(self.source_destination_dict.keys())

    def handle_syscommand(self, data):
        message = RosUnitySysCommand().deserialize(data)
        function = getattr(self.syscommands, message.command)
        if function is None:
            self.send_unity_error(
                "Don't understand SysCommand.'{}'({})".format(message.command, message.params_json)
            )
            return
        else:
            params = json.loads(message.params_json)
            function(**params)


class SysCommands:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server

    def subscribe(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't subscribe to a blank topic name! SysCommand.subscribe({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.subscribe - Unknown message class '{}'".format(message_name)
            )
            return

        rospy.loginfo("RegisterSubscriber({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.source_destination_dict:
            self.tcp_server.source_destination_dict[topic].unregister()

        self.tcp_server.source_destination_dict[topic] = RosSubscriber(
            topic, message_class, self.tcp_server
        )

    def publish(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't publish to a blank topic name! SysCommand.publish({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.publish - Unknown message class '{}'".format(message_name)
            )
            return

        rospy.loginfo("RegisterPublisher({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.source_destination_dict:
            self.tcp_server.source_destination_dict[topic].unregister()

        self.tcp_server.source_destination_dict[topic] = RosPublisher(
            topic, message_class, queue_size=10
        )

    def ros_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        rospy.loginfo("RegisterRosService({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.source_destination_dict:
            self.tcp_server.source_destination_dict[topic].unregister()

        self.tcp_server.source_destination_dict[topic] = RosService(topic, message_class)

    def unity_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        rospy.loginfo("RegisterUnityService({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.source_destination_dict:
            self.tcp_server.source_destination_dict[topic].unregister()

        self.tcp_server.source_destination_dict[topic] = UnityService(
            topic.encode("ascii"), message_class, self.tcp_server
        )


def resolve_message_name(name, extension="msg"):
    try:
        names = name.split("/")
        module_name = names[0]
        class_name = names[1]
        importlib.import_module(module_name + "." + extension)
        module = sys.modules[module_name]
        if module is None:
            rospy.logerr("Failed to resolve module {}".format(module_name))
        module = getattr(module, extension)
        if module is None:
            rospy.logerr("Failed to resolve module {}.{}".format(module_name, extension))
        module = getattr(module, class_name)
        if module is None:
            rospy.logerr(
                "Failed to resolve module {}.{}.{}".format(module_name, extension, class_name)
            )
        return module
    except (IndexError, KeyError, AttributeError, ImportError) as e:
        rospy.logerr("Failed to resolve message name: {}".format(e))
        return None
