from unittest import mock

import rospy
from ros_tcp_endpoint.publisher import RosPublisher


@mock.patch.object(rospy, "Publisher")
def test_publisher_send(mock_ros):
    mock_tcp_server = mock.Mock()
    publisher = RosPublisher("color", mock.Mock(), mock_tcp_server)
    publisher.send("test data")
    publisher.msg.deserialize.assert_called_once()
    publisher.pub.publish.assert_called_once()


@mock.patch.object(rospy, "Publisher")
def test_publisher_unregister(mock_ros):
    mock_tcp_server = mock.Mock()
    publisher = RosPublisher("color", mock.Mock(), mock_tcp_server)
    publisher.unregister()
    publisher.pub.unregister.assert_called_once()
