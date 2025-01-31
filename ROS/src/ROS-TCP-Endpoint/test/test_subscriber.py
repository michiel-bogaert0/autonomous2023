# flake8: noqa
from unittest import mock

import rospy
from ros_tcp_endpoint.subscriber import RosSubscriber


@mock.patch.object(rospy, "Subscriber")
def test_subscriber_send(mock_ros):
    mock_tcp_server = mock.Mock()
    subscriber = RosSubscriber("color", mock.Mock(), mock_tcp_server)
    assert subscriber.node_name == "color_RosSubscriber"
    subscriber.send("test data")
    mock_tcp_server.send_unity_message.assert_called_once()


@mock.patch.object(rospy, "Subscriber")
def test_subscriber_unregister(mock_ros):
    mock_tcp_server = mock.Mock()
    subscriber = RosSubscriber("color", mock.Mock(), mock_tcp_server)
    assert subscriber.node_name == "color_RosSubscriber"
    subscriber.unregister()
    subscriber.sub.unregister.assert_called_once()
