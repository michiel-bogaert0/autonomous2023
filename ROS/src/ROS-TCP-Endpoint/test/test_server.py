# flake8: noqa
import importlib
import sys
from unittest import mock

import ros_tcp_endpoint
import rospy
from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.server import SysCommands


@mock.patch("socket.socket")
@mock.patch("ros_tcp_endpoint.server.rospy")
def test_server_constructor(mock_ros, mock_socket):
    mock_ros.get_param = mock.Mock(return_value="127.0.0.1")
    server = TcpServer("test-tcp-server")
    assert server.node_name == "test-tcp-server"
    assert server.tcp_ip == "127.0.0.1"
    assert server.buffer_size == 1024
    assert server.connections == 10


def test_start_server():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    assert server.tcp_ip == "127.0.0.1"
    assert server.tcp_port == 10000
    assert server.connections == 10
    server.start()


def test_unity_service_empty_topic_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    system_cmds = SysCommands(server)
    result = system_cmds.unity_service("", "test message")
    assert result is None


def test_unity_service_resolve_message_name_failure():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    system_cmds = SysCommands(server)
    result = system_cmds.unity_service("get_pos", "unresolvable message")
    assert result is None


@mock.patch.object(rospy, "Service")
@mock.patch.object(
    SysCommands,
    "resolve_message_name",
    return_value="unity_interfaces.msg/RosUnitySrvMessage",
)
def test_unity_service_resolve_news_service(mock_resolve_message, mock_ros_service):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    assert server.ros_services_table == {}
    system_cmds = SysCommands(server)
    result = system_cmds.unity_service(
        "get_pos", "unity_interfaces.msg/RosUnitySrvMessage"
    )
    mock_ros_service.assert_called_once
    assert result is None


@mock.patch.object(rospy, "Service")
@mock.patch.object(
    SysCommands,
    "resolve_message_name",
    return_value="unity_interfaces.msg/RosUnitySrvMessage",
)
def test_unity_service_resolve_existing_service(mock_resolve_message, mock_ros_service):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    server.ros_services = {"get_pos": mock.Mock()}
    system_cmds = SysCommands(server)
    result = system_cmds.unity_service(
        "get_pos", "unity_interfaces.msg/RosUnitySrvMessage"
    )
    mock_ros_service.assert_called_once
    assert result is None


@mock.patch.object(sys, "modules", return_value="unity_interfaces.msg")
@mock.patch.object(importlib, "import_module")
def test_resolve_message_name(mock_import_module, mock_sys_modules):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    msg_name = "unity_interfaces.msg/UnityColor.msg"
    result = SysCommands(server).resolve_message_name(msg_name)
    mock_import_module.assert_called_once
    mock_sys_modules.assert_called_once
    assert result is not None


@mock.patch.object(rospy, "Publisher")
def test_publish_add_new_topic(mock_ros_publisher):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).publish("object_pos_topic", "std_msgs/Bool")
    assert server.publishers_table != {}
    mock_ros_publisher.assert_called_once


@mock.patch.object(rospy, "Publisher")
def test_publish_existing_topic(mock_ros_publisher):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    server.publishers_table = {"object_pos_topic": mock.Mock()}
    result = SysCommands(server).publish("object_pos_topic", "std_msgs/Bool")
    assert server.publishers_table["object_pos_topic"] is not None
    mock_ros_publisher.assert_called_once


def test_publish_empty_topic_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).publish("", "pos")
    assert result is None
    assert server.publishers_table == {}


def test_publish_empty_message_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).publish("test-topic", "")
    assert result is None
    assert server.publishers_table == {}


@mock.patch.object(rospy, "Subscriber")
@mock.patch.object(
    SysCommands, "resolve_message_name", return_value="unity_interfaces.msg/Pos"
)
def test_subscribe_to_new_topic(mock_resolve_msg, mock_ros_subscriber):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).subscribe("object_pos_topic", "pos")
    assert server.subscribers_table != {}
    mock_ros_subscriber.assert_called_once


@mock.patch.object(rospy, "Subscriber")
@mock.patch.object(
    SysCommands, "resolve_message_name", return_value="unity_interfaces.msg/Pos"
)
def test_subscribe_to_existing_topic(mock_resolve_msg, mock_ros_subscriber):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    server.subscribers_table = {"object_pos_topic": mock.Mock()}
    result = SysCommands(server).subscribe("object_pos_topic", "pos")
    assert server.subscribers_table["object_pos_topic"] is not None
    mock_ros_subscriber.assert_called_once


def test_subscribe_to_empty_topic_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).subscribe("", "pos")
    assert result is None
    assert server.subscribers_table == {}


def test_subscribe_to_empty_message_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).subscribe("test-topic", "")
    assert result is None
    assert server.subscribers_table == {}


@mock.patch.object(rospy, "ServiceProxy")
@mock.patch.object(SysCommands, "resolve_message_name")
def test_ros_service_new_topic(mock_resolve_msg, mock_ros_service):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).ros_service("object_pos_topic", "pos")
    assert server.ros_services_table != {}
    mock_ros_service.assert_called_once


@mock.patch.object(rospy, "ServiceProxy")
@mock.patch.object(SysCommands, "resolve_message_name")
def test_ros_service_existing_topic(mock_resolve_msg, mock_ros_service):
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    server.ros_services_table = {"object_pos_topic": mock.Mock()}
    result = SysCommands(server).ros_service("object_pos_topic", "pos")
    assert server.ros_services_table["object_pos_topic"] is not None
    mock_ros_service.assert_called_once


def test_ros_service_empty_topic_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).ros_service("", "pos")
    assert result is None
    assert server.ros_services_table == {}


def test_ros_service_empty_message_should_return_none():
    server = TcpServer(node_name="test-tcp-server", tcp_ip="127.0.0.1", tcp_port=10000)
    result = SysCommands(server).ros_service("test-topic", "")
    assert result is None
    assert server.ros_services_table == {}
