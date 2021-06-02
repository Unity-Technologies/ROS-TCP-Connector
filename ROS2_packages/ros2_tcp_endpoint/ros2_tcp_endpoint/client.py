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

import rclpy
import struct

import threading

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from .exceptions import TopicOrServiceNameDoesNotExistError
from unity_interfaces.msg import RosUnitySrvMessage


class ClientThread(threading.Thread):
    """
    Thread class to read all data from a connection and pass along the data to the
    desired source.
    """
    def __init__(self, conn, tcp_server, incoming_ip, incoming_port):
        """
        Set class variables
        Args:
            conn:
            source_destination_dict: dictionary of destination name to RosCommunicator class
        """
        self.conn = conn
        self.tcp_server = tcp_server
        self.incoming_ip = incoming_ip
        self.incoming_port = incoming_port
        threading.Thread.__init__(self)

    @staticmethod
    def recvall(conn, size, flags=0):
        """
        Receive exactly bufsize bytes from the socket.
        """
        buffer = bytearray(size)
        view = memoryview(buffer)
        pos = 0
        while pos < size:
            read = conn.recv_into(view[pos:], size - pos, flags)
            if not read:
                raise IOError("No more data available")
            pos += read
        return bytes(buffer)

    @staticmethod
    def read_int32(conn):
        """
        Reads four bytes from socket connection and unpacks them to an int

        Returns: int

        """
        raw_bytes = ClientThread.recvall(conn, 4)
        num = struct.unpack('<I', raw_bytes)[0]
        return num

    def read_string(self):
        """
        Reads int32 from socket connection to determine how many bytes to
        read to get the string that follows. Read that number of bytes and
        decode to utf-8 string.

        Returns: string

        """
        str_len = ClientThread.read_int32(self.conn)

        str_bytes = ClientThread.recvall(self.conn, str_len)
        decoded_str = str_bytes.decode('utf-8')

        return decoded_str

    def read_message(self):
        """
        Decode destination and full message size from socket connection.
        Grab bytes in chunks until full message has been read.
        """
        data = b''

        destination = self.read_string()
        full_message_size = ClientThread.read_int32(self.conn)

        while len(data) < full_message_size:
            # Only grabs max of 1024 bytes TODO: change to TCPServer's buffer_size
            grab = 1024 if full_message_size - len(data) > 1024 else full_message_size - len(data)
            packet = ClientThread.recvall(self.conn, grab)

            if not packet:
                self.tcp_server.get_logger().error("No packets...")
                break

            data += packet

        if full_message_size > 0 and not data:
            self.tcp_server.get_logger().error("No data for a message size of {}, breaking!".format(full_message_size))
            return

        destination = destination.rstrip('\x00')
        return destination, data

    @staticmethod
    def serialize_message(destination, message):
        """
        Serialize a destination and message class.

        Args:
            destination: name of destination
            message:     message class to serialize

        Returns:
            serialized destination and message as a list of bytes
        """
        dest_bytes = destination.encode('utf-8')
        length = len(dest_bytes)
        dest_info = struct.pack('<I%ss' % length, length, dest_bytes)

        serial_response = serialize_message(message)

        msg_length = struct.pack('<I', len(serial_response))
        serialized_message = dest_info + msg_length + serial_response

        return serialized_message

    def run(self):
        """
        Read a message and determine where to send it based on the source_destination_dict
         and destination string. Then send the read message.

        If there is a response after sending the serialized data, assume it is a
        ROS service response.

        Message format is expected to arrive as
            int: length of destination bytes
            str: destination. Publisher topic, Subscriber topic, Service name, etc
            int: size of full message
            msg: the ROS msg type as bytes

        """
        self.tcp_server.get_logger().info("Connection from {}".format(self.incoming_ip))
        halt_event = threading.Event()
        self.tcp_server.unity_tcp_sender.start_sender(self.conn, halt_event)
        try:
            while not halt_event.is_set():
                destination, data = self.read_message()

                if destination == '':
                    #ignore this keepalive message, listen for more
                    pass
                elif destination == '__syscommand':
                    #handle a system command, such as registering new topics
                    self.tcp_server.handle_syscommand(data)
                elif destination == '__srv':
                    # a __srv message from Unity is either a request to a ros service, or a response from a unity service
                    srv_message = deserialize_message(data, RosUnitySrvMessage())
                    if not srv_message.is_request:
                        self.tcp_server.send_unity_service_response(srv_message.srv_id, bytes(srv_message.payload))
                        continue # this was an outgoing response sent by a Unity service, nothing else to do here
                    elif srv_message.topic == '__topic_list':
                        response = self.tcp_server.topic_list(data)
                    elif srv_message.topic not in self.tcp_server.source_destination_dict.keys():
                        error_msg = "Service destination '{}' is not registered! Known topics are: {} "\
                            .format(srv_message.topic, self.tcp_server.source_destination_dict.keys())
                        self.tcp_server.send_unity_error(error_msg)
                        self.tcp_server.get_logger().error(error_msg)
                        # TODO: send a response to Unity anyway?
                        continue                   
                    else:
                        ros_communicator = self.tcp_server.source_destination_dict[srv_message.topic]
                        response = ros_communicator.send(bytes(srv_message.payload))
                        if not response:
                            error_msg = "No response data from service '{}'!".format(srv_message.topic)
                            self.tcp_server.send_unity_error(error_msg)
                            self.tcp_server.get_logger().error(error_msg)
                            # TODO: send a response to Unity anyway?
                            continue
                    
                    # got an incoming response from the service, send it back to Unity
                    serial_response = serialize_message(response)
                    response_message = RosUnitySrvMessage(srv_id=srv_message.srv_id, is_request=False, topic='', payload=serial_response)
                    self.tcp_server.unity_tcp_sender.send_unity_message("__srv", response_message)
                elif destination in self.tcp_server.source_destination_dict:
                    ros_communicator = self.tcp_server.source_destination_dict[destination]
                    response = ros_communicator.send(data)
                else:
                    error_msg = "Topic '{}' is not registered! Known topics are: {} "\
                        .format(destination, self.tcp_server.source_destination_dict.keys())
                    self.tcp_server.send_unity_error(error_msg)
                    self.tcp_server.get_logger().error(error_msg)
        except IOError as e:
            self.tcp_server.get_logger().error("Exception: {}".format(e))
        finally:
            halt_event.set()
            self.conn.close()
            self.tcp_server.get_logger().info("Disconnected from {}".format(self.incoming_ip));
