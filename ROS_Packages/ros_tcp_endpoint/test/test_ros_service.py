from unittest import mock
from ros_tcp_endpoint.service import RosService
import rospy


@mock.patch.object(rospy, "ServiceProxy")
def test_subscriber_send(mock_ros):
    ros_service = RosService("color", mock.Mock())
    service_response = ros_service.send("test data")
    ros_service.srv_class.deserialize.assert_called_once()
    assert service_response is not None


@mock.patch.object(rospy, "ServiceProxy")
def test_subscriber_unregister(mock_ros):
    ros_service = RosService("color", mock.Mock())
    ros_service.unregister()
    ros_service.srv.close.assert_called_once()
