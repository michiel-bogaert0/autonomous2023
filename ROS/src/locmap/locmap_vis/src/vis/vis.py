import rospy
from ugr_msgs.msg import Observations, Particles
from visualization_msgs.msg import Marker, MarkerArray


class LocMapVis:
    def __init__(self, colors=None) -> None:
        self.colors = (
            [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
            if colors is None
            else colors
        )

        self.id = 0

    def delete_markerarray(self, namespace):
        """
        Deletes all markes of a given namespace

        Args:
            namespace: the namespace to remove the markers from

        Returns:
            MakerArray message
        """

        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = namespace
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)

        return marker_array_msg

    def particles_to_markerarray(
        self, particles: Particles, namespace, lifetime, color, persist=False
    ):
        """
            Takes in an Particles message and produces the corresponding MarkerArary message

        Args:
            - observations: the message to visualize
            - namespace: the namespace of the MarkerArray
            - lifetime: the lifetime of the markers
            - color: can be 'r', 'g', 'b'. Determines the base color of the weight-based gradient
            - persist: set to true if the markers need to persis (this is different than lifetime=0)

        Returns:
            MakerArray message
        """

        max_weight = 0

        for part in particles.particles:
            max_weight = max(max_weight, part.weight)

        marker_array = MarkerArray()

        for i, part in enumerate(particles.particles):
            marker = Marker()

            marker.header = particles.header
            marker.ns = namespace

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            if persist:
                marker.id = i + self.id
                self.id += 1
            else:
                marker.id = i

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = part.position.x
            marker.pose.position.y = part.position.y

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.02

            marker.color.r = 1 if color == "r" else part.weight / max_weight
            marker.color.g = 1 if color == "g" else part.weight / max_weight
            marker.color.b = 1 if color == "b" else part.weight / max_weight
            marker.color.a = 1

            marker.lifetime = rospy.Duration(lifetime)

            marker_array.markers.append(marker)

        return marker_array

    def observations_to_markerarray(
        self, observations: Observations, namespace, lifetime, persist=False
    ):
        """
        Takes in an Observations message and produces the corresponding MarkerArary message

        Args:
            - observations: the message to visualize
            - namespace: the namespace of the MarkerArray
            - lifetime: the lifetime of the markers
            - persist: set to true if the markers need to persis (this is different than lifetime=0)

        Returns:
            MakerArray message
        """

        marker_array = MarkerArray()

        for i, obs in enumerate(observations.observations):
            marker = Marker()

            marker.header = observations.header
            marker.ns = namespace

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            if persist:
                marker.id = i + self.id
                self.id += 1
            else:
                marker.id = i

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = obs.location.x
            marker.pose.position.y = obs.location.y

            marker.scale.x = 0.228
            marker.scale.y = 0.228
            marker.scale.z = 0.325

            color = self.colors[obs.observation_class]

            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1

            marker.lifetime = rospy.Duration(lifetime)

            marker_array.markers.append(marker)

        return marker_array
