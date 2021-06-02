from unittest import mock
from unittest.mock import Mock
from ros_tcp_endpoint.client import ClientThread
from ros_tcp_endpoint.server import TcpServer
import sys
import threading


def test_client_thread_initialization():
    tcp_server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    mock_conn = mock.Mock()
    client_thread = ClientThread(mock_conn, tcp_server, "127.0.0.1", 10000)
    assert client_thread.tcp_server.node_name == "test-tcp-server"
    assert client_thread.incoming_ip == "127.0.0.1"
    assert client_thread.incoming_port == 10000


def test_recvall_should_read_bytes_exact_size():
    mock_conn = mock.Mock()
    mock_conn.recv_into.return_value = 1
    result = ClientThread.recvall(mock_conn, 8)
    assert result == b"\x00\x00\x00\x00\x00\x00\x00\x00"


@mock.patch.object(ClientThread, "recvall", return_value=b"\x01\x00\x00\x00")
def test_read_int32_should_parse_int(mock_recvall):
    mock_conn = mock.Mock()
    result = ClientThread.read_int32(mock_conn)
    mock_recvall.assert_called_once
    assert result == 1


@mock.patch.object(ClientThread, "read_int32", return_value=4)
@mock.patch.object(ClientThread, "recvall", return_value=b"Hello world!")
def test_read_string_should_decode(mock_recvall, mock_read_int):
    mock_conn = mock.Mock()
    result = ClientThread.read_string(mock_conn)
    mock_recvall.assert_called_once
    mock_read_int.assert_called_once
    assert result == "Hello world!"


@mock.patch.object(ClientThread, "read_string", return_value="__srv")
@mock.patch.object(ClientThread, "read_int32", return_value=4)
@mock.patch.object(ClientThread, "recvall", return_value=b"Hello world!")
def test_read_message_return_destination_data(mock_recvall, mock_read_int, mock_read_str):
    mock_conn = mock.Mock()
    result = ClientThread.read_message(mock_conn)
    assert result == ("__srv", b"Hello world!")
