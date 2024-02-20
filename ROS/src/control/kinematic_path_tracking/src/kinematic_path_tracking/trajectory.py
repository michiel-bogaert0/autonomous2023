import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.fixture import AutonomousMission, ROSNode
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow

    Args:
        - tf_buffer {tf2_ros.Buffer}: tf2 buffer to transform the path to the current base_link_frame.
                                      Needed because when "configuring" the trajectory is reset, but might cause isues
                                        when the transform is not yet available (because of newly created buffer).
    """

    def __init__(self, tf_buffer):
        self.closest_index = 0
        self.current_position_index = 0
        self.points = np.array([])
        self.target = np.array([0, 0])

        # True for trackdrive/autocross, False for skidpad/acceleration
        self.change_index = rospy.get_param("~change_index", True)

        # Transformations
        self.tf_buffer = tf_buffer
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        # For skidpad/acceleration use ugr/map, for trackdrive/autocross use ugr/car_odom
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.time_source = rospy.Time(0)

        self.path = Path()

    def set_path(self, path):
        self.path = path
        self.time_source = path.header.stamp

        current_path = np.zeros((0, 2))
        for pose in path.poses:
            current_path = np.vstack(
                (current_path, [pose.pose.position.x, pose.pose.position.y])
            )

        self.points = current_path

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

        self.time_source = self.trans.header.stamp

        # Transform path
        self.path = ROSNode.do_transform_path(self.path, self.trans)

        # save points for next transform
        self.points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in self.path.poses]
        )
        return self.points

    def calculate_transversal_error(self):
        """
        Calculates the transversal error of the car to the path

        Returns:
            transverse_error {float}: transverse error
            heading_error {float}: heading error (relative to tangent of path)
        """

        self.__preprocess_path__()

        if len(self.path_blf) == 0:
            return 0, 0

        # Catch up to current position (if needed) by checking if distance to next point on path is increasing
        prev_distance = (0 - self.path_blf[self.closest_index][0]) ** 2 + (
            0 - self.path_blf[self.closest_index][1]
        ) ** 2
        for _ in range(len(self.path_blf) + 1):
            tmp_index = (self.closest_index + 1) % len(self.path_blf)
            target_x = self.path_blf[tmp_index][0]
            target_y = self.path_blf[tmp_index][1]

            # Current position is [0,0] in base_link_frame
            distance = (0 - target_x) ** 2 + (0 - target_y) ** 2

            if distance > prev_distance:
                break

            prev_distance = distance

            self.closest_index = tmp_index

        tangent = np.array(
            [
                self.path_blf[(self.closest_index + 1) % len(self.path_blf)][0]
                - self.path_blf[self.closest_index][0],
                self.path_blf[(self.closest_index + 1) % len(self.path_blf)][1]
                - self.path_blf[self.closest_index][1],
            ]
        )
        normalized_tangent = tangent / np.linalg.norm(tangent)

        target_vector = np.array(
            [
                self.path_blf[self.closest_index][0],
                self.path_blf[self.closest_index][1],
            ]
        )

        transverse_error = np.cross(normalized_tangent, target_vector)
        heading_error = np.arctan2(normalized_tangent[1], normalized_tangent[0])

        self.target_vector = target_vector

        return transverse_error, heading_error

    def __preprocess_path__(self):
        """
        preprocesses a path, given in points (N,2) from base_link_frame to base_link_frame at current time
        Also calculates the closest index to the current position
        """

        # Transfom path to most recent blf
        self.path_blf = self.transform_blf()

        # No path received
        if len(self.path_blf) == 0:
            return 0, 0

        # Only calculate closest index as index of point with smallest distance to current position if working in trakdrive/autocross
        self.current_position_index = (
            np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
            if self.change_index
            else self.closest_index
        )
        self.closest_index = self.current_position_index

    def calculate_target_point(self, minimal_distance, mission):
        """
        Calculates a target point by traversing the path
        Returns the first points that matches the conditions given by minimal_distance

        Args:
            minimal_distance: minimal lookahead distance

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
        """

        self.__preprocess_path__()

        if len(self.path_blf) == 0:
            return 0, 0, 0

        distance = 0.0

        for _ in range(len(self.path_blf) + 1):
            target_x = self.path_blf[self.closest_index][0]
            target_y = self.path_blf[self.closest_index][1]

            if (
                mission == AutonomousMission.SKIDPAD
                or mission == AutonomousMission.ACCELERATION
            ):
                # For skidpad/acc, minimal distance has to be defined as direct distance to the car (0,0)
                distance = np.sqrt(target_x**2 + target_y**2)
            elif (
                mission == AutonomousMission.TRACKDRIVE
                or mission == AutonomousMission.AUTOCROSS
            ):
                # For trackdrive/autocross, minimal distance is defined over the path
                target_x_pp = self.path_blf[
                    (self.closest_index + 1) % len(self.path_blf)
                ][0]
                target_y_pp = self.path_blf[
                    (self.closest_index + 1) % len(self.path_blf)
                ][1]

                distance += np.sqrt(
                    (target_x_pp - target_x) ** 2 + (target_y_pp - target_y) ** 2
                )

            if distance > minimal_distance:
                self.target = np.array([target_x, target_y])
                return (self.target[0], self.target[1], self.time_source)

            self.closest_index = (self.closest_index + 1) % len(self.path_blf)

            # If no new target point, return last target point
            if self.closest_index == self.current_position_index:
                pose = PoseStamped(
                    header=Header(
                        frame_id=self.base_link_frame, stamp=self.trans.header.stamp
                    )
                )

                pose.pose.position.x = self.target[0]
                pose.pose.position.y = self.target[1]

                pose_t = do_transform_pose(pose, self.trans)

                self.target = np.array([pose_t.pose.position.x, pose_t.pose.position.y])
                return (self.target[0], self.target[1], self.time_source)
