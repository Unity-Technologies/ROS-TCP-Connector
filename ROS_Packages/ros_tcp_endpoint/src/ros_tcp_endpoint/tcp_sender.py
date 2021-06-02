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
import socket
import time
import threading
import struct
from .client import ClientThread
from .thread_pauser import ThreadPauser
from unity_interfaces.msg import RosUnityError
from unity_interfaces.msg import RosUnitySrvMessage
from io import BytesIO

# queue module was renamed between python 2 and 3
try:
    from queue import Queue
    from queue import Empty
except:
    from Queue import Queue
    from Queue import Empty


class UnityTcpSender:
    """
    Connects and sends messages to the server on the Unity side.
    """

    def __init__(self):
        # if we have a valid IP at this point, it was overridden locally so always use that
        self.sender_id = 1
        self.time_between_halt_checks = 5

        # Each sender thread has its own queue: this is always the queue for the currently active thread.
        self.queue = None
        self.queue_lock = threading.Lock()

        # variables needed for matching up unity service requests with responses
        self.next_srv_id = 1001
        self.srv_lock = threading.Lock()
        self.services_waiting = {}

    def send_unity_error(self, error):
        self.send_unity_message("__error", RosUnityError(error))

    def send_unity_message(self, topic, message):
        serialized_message = ClientThread.serialize_message(topic, message)
        if self.queue is not None:
            self.queue.put(serialized_message)

    def send_unity_service(self, topic, service_class, request):
        request_bytes = BytesIO()
        request.serialize(request_bytes)
        thread_pauser = ThreadPauser()

        with self.srv_lock:
            srv_id = self.next_srv_id
            self.next_srv_id += 1
            self.services_waiting[srv_id] = thread_pauser

        payload = request_bytes.getvalue()
        srv_message = RosUnitySrvMessage(srv_id, True, topic, payload)
        serialized_message = ClientThread.serialize_message("__srv", srv_message)
        if self.queue is not None:
            self.queue.put(serialized_message)

        # rospy starts a new thread for each service request,
        # so it won't break anything if we sleep now while waiting for the response
        thread_pauser.sleep_until_resumed()

        response = service_class._response_class().deserialize(thread_pauser.result)
        return response

    def send_unity_service_response(self, srv_id, data):
        thread_pauser = None
        with self.srv_lock:
            thread_pauser = self.services_waiting[srv_id]
            del self.services_waiting[srv_id]

        thread_pauser.resume_with_result(data)

    def start_sender(self, conn, halt_event):
        sender_thread = threading.Thread(
            target=self.sender_loop, args=(conn, self.sender_id, halt_event)
        )
        self.sender_id += 1

        # Exit the server thread when the main thread terminates
        sender_thread.daemon = True
        sender_thread.start()

    def sender_loop(self, conn, tid, halt_event):
        s = None
        local_queue = Queue()
        # send an empty message to confirm connection
        # minimal message: 4 zero bytes for topic length 0, 4 zero bytes for payload length 0
        local_queue.put(b"\0\0\0\0\0\0\0\0")
        with self.queue_lock:
            self.queue = local_queue

        try:
            while not halt_event.is_set():
                try:
                    item = local_queue.get(timeout=self.time_between_halt_checks)
                except Empty:
                    # I'd like to just wait on the queue, but we also need to check occasionally for the connection being closed
                    # (otherwise the thread never terminates.)
                    continue

                # print("Sender {} sending an item".format(tid))

                try:
                    conn.sendall(item)
                except Exception as e:
                    rospy.logerr("Exception on Send {}".format(e))
                    break
        finally:
            halt_event.set()
            with self.queue_lock:
                if self.queue is local_queue:
                    self.queue = None
