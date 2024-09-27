#!/usr/bin/python3
import os

import rospkg
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from node_fixture.fixture import NodeManagingStatesEnum
from node_fixture.managed_node import ManagedNode
from node_fixture.srv import GetNodeState, SetNodeState


def set_state(name: str, state: str):
    """
    Set the state of a node.

    Args:
        name (str): The name of the node.
        state (str): The state to set the node to.
    """
    rospy.wait_for_service(f"/node_managing/{name}/set", timeout=0.5)
    return rospy.ServiceProxy(f"/node_managing/{name}/set", SetNodeState)(state)


def set_state_active(name: str):
    """
    Set the state of a node to active.

    Args:
        name (str): The name of the node.
    """
    return set_state(name, NodeManagingStatesEnum.ACTIVE)


def set_state_inactive(name: str):
    """
    Set the state of a node to inactive.

    Args:
        name (str): The name of the node.
    """
    return set_state(name, NodeManagingStatesEnum.INACTIVE)


def set_state_unconfigured(name: str):
    """
    Set the state of a node to unconfigured.

    Args:
        name (str): The name of the node.
    """
    return set_state(name, NodeManagingStatesEnum.UNCONFIGURED)


def set_state_finalized(name: str):
    """
    Set the state of a node to finalized.

    Args:
        name (str): The name of the node.
    """
    return set_state(name, NodeManagingStatesEnum.FINALIZED)


def configure_node(name: str):
    """
    Set the state of a node to unconfigured and then to inactive.

    Args:
        name (str): The name of the node.
    """

    set_state_result = True

    rospy.wait_for_service(f"/node_managing/{name}/get", timeout=0.5)
    data = rospy.ServiceProxy(f"/node_managing/{name}/get", GetNodeState)()
    if data.state == NodeManagingStatesEnum.ACTIVE:
        set_state_result = set_state_result and set_state_inactive(name)
        set_state_result = set_state_result and set_state_unconfigured(name)
    elif data.state == NodeManagingStatesEnum.INACTIVE:
        set_state_result = set_state_result and set_state_unconfigured(name)

    set_state_result = set_state_result and set_state_inactive(name)

    return set_state_result


def load_params(mission: str) -> None:
    """
    Load parameters from a YAML file based the mission.
    Also takes the car name from the /car parameter.

    Args:
        mission (str): The name of the mission.

    Returns:
        None
    """
    pkg_path = rospkg.RosPack().get_path("ugr_launch")
    car = rospy.get_param("/car")
    filename = f"{mission}.yaml"
    yaml_path = os.path.join(pkg_path, "config/", car, filename)
    with open(yaml_path, "r") as f:
        dic = yaml.safe_load(f)
    if dic is None:
        rospy.logerr(f"Could not load parameters from {yaml_path}")
        return
    for param_name, param_value in get_params(dic):
        param_name = "/" + param_name
        rospy.set_param(param_name, param_value)


def get_params(d: dict):
    """
    Recursively iterates through a dictionary and yields key-value pairs.

    Args:
        d (dict): The dictionary to iterate through.

    Yields:
        tuple: A tuple containing the key-value pair.

    """
    for key, value in d.items():
        if isinstance(value, dict):
            for lkey, lvalue in get_params(value):
                yield (key + "/" + lkey, lvalue)
        else:
            yield (key, value)


class NodeManager(ManagedNode):
    def __init__(self, name: str, default_state=NodeManagingStatesEnum.UNCONFIGURED):
        """
        This class is a node manager for the node_fixture package. It is responsible for managing the states of nodes

        Args:
            name (str): The name of the node (this node, given to ManagedNode).
            default_state (str): The default state of the node.
        """

        super().__init__(name, default_state)

        self.nodes_to_monitor = set([])
        self.timers = {}
        self.health_msgs = {}
        self.unhealty_status_self_inflicted = False

        self.health_array = DiagnosticArray()

        # Params
        self.timeout = rospy.get_param("~timeout", 1)
        self.startup_timeout = rospy.get_param("~startup_timeout", 10)
        self.healthdiagrate = rospy.Rate(rospy.get_param("~healthdiagrate", 5))

        # Pubs and subs
        rospy.Subscriber("/health/nodes", DiagnosticStatus, self.handle_health)
        self.health_pub = rospy.Publisher(
            "/health/diagnostics", DiagnosticArray, queue_size=1
        )

        # Set health to warning
        self.set_health(level=DiagnosticStatus.WARN, message="Initializing")

    def handle_health(self, msg: DiagnosticStatus):
        """
        Handle the incoming health messages from the nodes. This is a callback function for the /health/nodes topic.

        Args:
            msg (DiagnosticStatus): The incoming health message.

        """

        if msg.name.startswith("healthchecks"):
            node = msg.hardware_id
            self.timers[node] = rospy.Time.now().to_sec()
            self.health_msgs[node] = msg

    def spin(self):
        """
        Custom spin function for the node manager.
        This function checks the health of the nodes on top of the normal stuff.
        """

        while not rospy.is_shutdown():
            self.spinOnce()
            self.check_health()

    def check_health(self):
        """
        Checks the health of the nodes. This function should be called in a loop.
        """

        # This function should be called in a loop
        # Basically checks health checks of all nodes that should be active
        # If something wrong occurs, this node will go into error or warn itself

        keyvalues = []

        # Add metadata to keyvalues about monitoring
        keyvalues.append(
            KeyValue(key="Monitored nodes", value=", ".join(self.nodes_to_monitor))
        )

        new_health_level = DiagnosticStatus.OK
        for node in self.nodes_to_monitor:
            # Initialize monitoring timer (with extra timeout as startup guard)
            # If node has not sent a heaertbeat yet, might still be starting up, set health to warning
            # Note that when the node fails starts up, the node manager will not go to error until startup timeout occurs
            if node not in self.timers.keys():
                self.timers[node] = rospy.Time.now().to_sec() + self.startup_timeout
                new_health_level = max(new_health_level, DiagnosticStatus.WARN)
                keyvalues.append(
                    KeyValue(key=node, value="No contact. Might still be starting up")
                )

            # Check if node is still alive, if not, set health to error
            if (
                node in self.timers.keys()
                and rospy.Time.now().to_sec() - self.timers[node] > self.timeout
            ):
                new_health_level = max(new_health_level, DiagnosticStatus.ERROR)
                keyvalues.append(
                    KeyValue(key=node, value="Lost contact. Node not healthy")
                )
            # This is fine, but check the "state" of the node
            elif node in self.health_msgs.keys():
                health_msg = self.health_msgs[node]
                if health_msg.values[0].value != "active":
                    new_health_level = DiagnosticStatus.ERROR
                    keyvalues.append(
                        KeyValue(key=node, value="Node not 'active' (yet)")
                    )

        # Of course the node manager should also act on reported errors and warnings
        # Final self health level is the highest level of all
        for node in self.nodes_to_monitor:
            if (
                node in self.health_msgs
                and self.health_msgs[node].level > new_health_level
            ):
                new_health_level = self.health_msgs[node].level

            # Also list ALL monitored nodes warnings and errors to keyvalues
            if (
                node in self.health_msgs
                and self.health_msgs[node].level > DiagnosticStatus.OK
            ):
                keyvalues.append(
                    KeyValue(key=node, value=self.health_msgs[node].message)
                )

        # Detect changes in health, to immediately publish this change
        publish = new_health_level != self.get_health_level()

        # Of course, when being in error itself, this should also propagate
        if (
            self.health.level != DiagnosticStatus.OK
            and not self.unhealty_status_self_inflicted
        ):
            new_health_level = max(new_health_level, self.health.level)

        if new_health_level == DiagnosticStatus.OK:
            self.set_health(
                level=new_health_level,
                message="All monitored nodes are healthy",
                values=keyvalues,
                publish=publish,
            )
            self.unhealty_status_self_inflicted = False
        else:
            message = (
                "There is an issue with at least one monitored node"
                if self.health.level == DiagnosticStatus.OK
                else self.health.message
            )
            self.set_health(
                level=new_health_level,
                message=message,
                values=keyvalues,
                publish=publish,
            )
            self.unhealty_status_self_inflicted = (
                self.health.level != DiagnosticStatus.OK
            )

        if publish:
            self.publish_health_diagnostics()

        # Publish health diagnostics periodically (even if no change)
        if self.healthdiagrate.remaining() < rospy.Duration(0):
            self.healthdiagrate.sleep()
            self.publish_health_diagnostics()

    def publish_health_diagnostics(self):
        """
        Publishes the health diagnostics to the /health/diagnostics topic.
        """

        # First make diagnostic array
        health_array = DiagnosticArray()
        health_array.header.stamp = rospy.Time.now()

        # Add itself as first item
        health_array.status.append(self.health)

        for node in self.nodes_to_monitor:
            if node in self.health_msgs.keys():
                health_array.status.append(self.health_msgs[node])

        # Then publish
        self.health_pub.publish(health_array)

    def get_node_management_params(self, new_state=None, old_state=None):
        """
        Get the node management parameters from the ROS parameter server.
        This is used to fetch which nodes should be "activated" and monitored in a specific state machine state (NOT node state!).

        Args:
            new_state (str): The new state of the state machine (not nodes)
            old_state (str): The old state of the state machine
        """
        node_management_param = rospy.get_param("~node_management")

        print(f"Old state: {old_state} -> New State: {new_state}")

        if "always" not in node_management_param.keys():
            node_management_param["always"] = []

        if new_state is not None and new_state not in node_management_param.keys():
            node_management_param[new_state] = []

        if old_state is not None and old_state not in node_management_param.keys():
            node_management_param[old_state] = []

        print(f"Params: {node_management_param}")

        return node_management_param

    def configure_nodes(self):
        """
        Configures the nodes that should be configured. This is done by setting the state of the nodes to unconfigured and then to inactive.
        Also sets the health of the node manager to OK if successful, or to ERROR if not.

        Returns:
            bool: True if successful, False if not.

        Throws:
            BaseException: If something goes wrong (typically when calling the set_state service or if the service is not available)
                           Own health state is set to error in this case.
        """

        try:
            # Get all nodes that should be configured
            node_management_param = self.get_node_management_params()

            nodes_to_configure = [
                item for state in node_management_param.values() for item in state
            ]
            nodes_to_configure = list(set(nodes_to_configure))

            for node in nodes_to_configure:
                if not configure_node(node):
                    raise BaseException(
                        f"Configure procedure failed for node {node}, raising error..."
                    )

            self.set_health(
                level=DiagnosticStatus.OK,
                message=f"Configured nodes: {nodes_to_configure}",
            )

            return True

        except BaseException as e:
            self.set_health(
                level=DiagnosticStatus.ERROR,
                message=f"Could not configure nodes. Error: {e}",
            )

            rospy.logerr(f"Could not configure nodes: {e}")

            return False

    def activate_nodes(self, new_state, old_state=None):
        """
        Activates the nodes that should be activated. This is done by setting the state of the nodes to unconfigured and then to active.
        Also sets the health of the node manager to OK if successful, or to ERROR if not.

        Returns:
            bool: True if successful, False if not.

        Throws:
            BaseException: If something goes wrong (typically when calling the set_state service or if the service is not available)
                           Own health state is set to error in this case.
        """

        try:
            node_management_param = self.get_node_management_params(
                new_state, old_state
            )

            # First activate nodes (that are not already active)
            for node in (
                node_management_param[new_state] + node_management_param["always"]
            ):
                if (
                    old_state is None
                    or node
                    not in node_management_param[old_state]
                    + node_management_param["always"]
                    or (
                        node in node_management_param["always"]
                        and node not in self.nodes_to_monitor
                    )
                ):
                    if not set_state_active(node):
                        raise BaseException(
                            f"Configure procedure failed for node {node}, raising error..."
                        )

                    self.nodes_to_monitor.add(node)

            # Then deactivate nodes (that are not in the new state
            if old_state is not None:
                for node in node_management_param[old_state]:
                    if (
                        node
                        not in node_management_param[new_state]
                        + node_management_param["always"]
                    ):
                        self.nodes_to_monitor.remove(node)
                        self.timers.pop(node, None)
                        self.health_msgs.pop(node, None)
                        if not set_state_inactive(node):
                            raise BaseException(
                                f"Configure procedure failed for node {node}, raising error..."
                            )

            self.set_health(
                level=DiagnosticStatus.OK, message=f"Activated nodes for {new_state}"
            )

        except BaseException as e:
            self.set_health(
                level=DiagnosticStatus.ERROR,
                message=f"Could not activate nodes, error: {e}",
            )
            rospy.logerr(f"Could not activate nodes for {new_state}: {e}")

            return False

        return True

    def unconfigure_nodes(self):
        """
        Unconfigures the nodes that should be unconfigured. This is done by setting the state of the nodes to inactive and then to unconfigured.
        """

        set_state_result = True
        for node in self.nodes_to_monitor:
            set_state_result = set_state_result and set_state_inactive(node)
            set_state_result = set_state_result and set_state_unconfigured(node)

        self.nodes_to_monitor = set([])
        self.timers = {}
        self.health_msgs = {}

        return set_state_result

    def finalize_nodes(self):
        """
        Finalizes the nodes that should be finalized. This is done by setting the state of the nodes to finalized.
        """

        set_state_result = True

        for node in self.nodes_to_monitor:
            set_state_result = set_state_result and set_state_finalized(node)

        self.nodes_to_monitor = set([])
        self.timers = {}
        self.health_msgs = {}

        return set_state_result
