import threading
from unittest import mock
from ros_tcp_endpoint.thread_pauser import ThreadPauser


@mock.patch.object(threading.Condition, "wait")
def test_sleep_until_resumed(mock_thread_wait):
    thread_pauser = ThreadPauser()
    thread_pauser.sleep_until_resumed()
    mock_thread_wait.assert_called_once


@mock.patch.object(threading.Condition, "notify")
def test_resume(mock_thread_notify):
    thread_pauser = ThreadPauser()
    thread_pauser.resume_with_result(mock.Mock)
    mock_thread_notify.assert_called_once
