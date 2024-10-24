# flake8: noqa
from unittest import mock

import rospy
from ros_tcp_endpoint.unity_service import UnityService


@mock.patch.object(rospy, "Service")
def test_unity_service_send(mock_ros_service):
    mock_tcp_server = mock.Mock()
    unity_service = UnityService("color", mock.Mock(), mock_tcp_server)
    assert unity_service.node_name == "color_service"
    unity_service.send("test data")
    mock_tcp_server.send_unity_service.assert_called_once()


@mock.patch.object(rospy, "Service")
def test_unity_service_unregister(mock_ros_service):
    mock_tcp_server = mock.Mock()
    unity_service = UnityService("color", mock.Mock(), mock_tcp_server)
    assert unity_service.node_name == "color_service"
    unity_service.unregister()
    unity_service.service.shutdown.assert_called_once()
