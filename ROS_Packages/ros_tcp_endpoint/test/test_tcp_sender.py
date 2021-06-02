import queue
import socket
from unittest import mock
import ros_tcp_endpoint


@mock.patch("ros_tcp_endpoint.tcp_sender.rospy")
def test_tcp_sender_constructor(mock_ros):
    tcp_sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    assert tcp_sender.sender_id == 1
    assert tcp_sender.time_between_halt_checks == 5
    assert tcp_sender.queue == None
    assert tcp_sender.next_srv_id == 1001
    assert tcp_sender.srv_lock != None
    assert tcp_sender.services_waiting == {}


@mock.patch("socket.socket")
@mock.patch.object(ros_tcp_endpoint.client.ClientThread, "serialize_message")
def test_send_unity_error_should_send_msg(mock_serialize_msg, mock_socket):
    sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    sender.send_unity_error("Test error")
    mock_serialize_msg.assert_called_once()


@mock.patch.object(ros_tcp_endpoint.client.ClientThread, "serialize_message")
def test_send_message_should_serialize_message(mock_serialize_msg):
    sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    sender.send_unity_message("test topic", "test msg")
    mock_serialize_msg.assert_called_once()


@mock.patch.object(ros_tcp_endpoint.thread_pauser.ThreadPauser, "sleep_until_resumed")
@mock.patch.object(ros_tcp_endpoint.client.ClientThread, "serialize_message")
def test_send_unity_service_should_serialize_ros_unity_srv(mock_serialize_msg, mock_thread_pauser):
    sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    sender.send_unity_service(mock.Mock(), mock.Mock(), mock.Mock())
    mock_serialize_msg.assert_called_once()
    # TODO: Test the scenario when the queue is not None
    assert sender.queue == None


@mock.patch("ros_tcp_endpoint.thread_pauser.ThreadPauser")
def test_send_unity_service_response_should_resume(mock_thread_pauser_class):
    sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    thread_pauser = mock_thread_pauser_class()
    sender.services_waiting = {"moveit": thread_pauser}
    sender.send_unity_service_response("moveit", "test data")
    thread_pauser.resume_with_result.assert_called_once()


def test_start_sender_should_increase_sender_id():
    sender = ros_tcp_endpoint.tcp_sender.UnityTcpSender()
    init_sender_id = 1
    assert sender.sender_id == init_sender_id
    sender.start_sender(mock.Mock(), mock.Mock())
    assert sender.sender_id == init_sender_id + 1
