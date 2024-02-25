import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.fixture import ROSNode
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow
    """

    def __init__(self):
        self.closest_index = 0
        self.points = np.array([])
        self.targets = []

        # True for trackdrive/autocross, False for skidpad/acceleration
        self.change_index = rospy.get_param("~change_index", True)

        # Transformations
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        # For skidpad/acceleration use ugr/map, for trackdrive/autocross use ugr/car_odom
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.time_source = rospy.Time(0)

        self.path = Path()

    def transform_blf(self):
        """
        transforms a path, given in points (N,2) from base_link_frame to base_link_frame at current time

        Args:
            None, all variables are class variables

        """
        self.trans = self.tf_buffer.lookup_transform_full(
            self.base_link_frame,
            rospy.Time(0),
            self.base_link_frame,
            self.time_source,
            self.world_frame,
            timeout=rospy.Duration(0.2),
        )

        # save time of last transform for next transform
        self.time_source = self.trans.header.stamp

        # Transform path
        self.path = ROSNode.do_transform_path(self.path, self.trans)

        # save points for next transform
        self.points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in self.path.poses]
        )
        return self.points

    def calculate_target_points(self, minimal_distances):
        """
        Calculates a target point by traversing the path
        Returns the first points that matches the conditions given by minimal_distance

        Args:
            minimal_distance: minimal lookahead distance

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
        """
        # transform path to most recent blf
        self.path_blf = self.transform_blf()

        # No path received
        if len(self.path_blf) == 0:
            return (0, 0)

        # Only calculate closest index as index of point with smallest distance to current position if working in trakdrive/autocross
        current_position_index = (
            np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
            if self.change_index
            else self.closest_index
        )
        self.closest_index = current_position_index

        targets = []
        indexes = []

        for i, dist in enumerate(minimal_distances):
            # Iterate until found
            for _ in range(len(self.path_blf) + 1):
                target_x = self.path_blf[self.closest_index][0]
                target_y = self.path_blf[self.closest_index][1]

                # Current position is [0,0] in base_link_frame
                distance = (0 - target_x) ** 2 + (0 - target_y) ** 2

                if distance > dist**2:
                    targets.append([target_x, target_y])
                    indexes.append(self.closest_index)
                    break

                self.closest_index = (self.closest_index + 1) % len(self.path_blf)

                # If no new point was found, use the last point
                if self.closest_index == current_position_index:
                    pose = PoseStamped(
                        header=Header(
                            frame_id=self.base_link_frame, stamp=self.trans.header.stamp
                        )
                    )

                    pose.pose.position.x = self.targets[i][0]
                    pose.pose.position.y = self.targets[i][1]

                    pose_t = do_transform_pose(pose, self.trans)

                    targets.append([pose_t.pose.position.x, pose_t.pose.position.y])
                    indexes.append(self.closest_index)
                    break
        self.targets = targets
        self.closest_index = indexes[0]
        return self.targets

    def get_reference_track(self, dt, N):
        self.path_blf = self.transform_blf()
        speed_target = rospy.get_param("/speed/target", 3.0)
        d = dt * speed_target
        distances = [(i + 1) * d for i in range(N)]  # 2 m/s^2 is max acceleration
        current_position_index = (
            np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
            if self.change_index
            else self.closest_index
        )
        # current_point = self.path_blf[current_position_index]
        # print(current_point)
        # distance_index = 0
        total_distance = 0
        distances_relative = []
        reference_path = []
        # Start from next point
        for i in range(
            current_position_index + 1, len(self.path_blf) + current_position_index
        ):
            i = i % len(self.path_blf)
            distances_relative.append(total_distance)
            total_distance += np.linalg.norm(self.path_blf[i] - self.path_blf[i - 1])
            # print(total_distance)
            # print(f"Point {i}: {self.path_blf[i]} with distance {total_distance}")

        for i in range(len(distances)):
            # Find first value in distances_relative that is greater than distance
            for j in range(len(distances_relative)):
                if distances_relative[j] >= distances[i]:
                    scaling = 1 - np.abs(
                        (distances[i] - distances_relative[j])
                        / (distances_relative[j] - distances_relative[j - 1])
                    )
                    # scaling=1
                    path_index = (current_position_index + j) % len(self.path_blf)
                    reference_path.append(
                        self.path_blf[path_index - 1]
                        + scaling
                        * (self.path_blf[path_index] - self.path_blf[path_index - 1])
                    )
                    # print(f"Point {j-1} and {j} with scaling {scaling} and distance {distances[i]} and relative distance {distances_relative[j]} results in point {reference_path[-1]}")
                    break
        # print(reference_path)
        return reference_path
        # print(reference_path)
