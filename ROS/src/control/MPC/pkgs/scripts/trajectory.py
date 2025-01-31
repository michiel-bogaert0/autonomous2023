import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture.fixture import ROSNode
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose

################################################
#                                              #
#   File outdated, will be fixed in other PR   #
#                                              #
################################################


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
        Calculates target points for all minimal distances given
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

    def get_reference_track(self, dt, N, actual_speed, debug=False):
        """
        Calculate the reference track based on the current path and the required velocity and maximum acceleration
        """
        self.path_blf = self.transform_blf()

        if len(self.path_blf) == 0:
            return []

        # Find target points based on required velocity and maximum acceleration
        speed_target = rospy.get_param("/speed/target", 3.0)

        max_acceleration = 2.0  # TODO: create param
        # calculate distances based on maximum acceleration and current speed
        distances = [
            (min(speed_target, actual_speed + dt * max_acceleration * i)) * i * dt
            for i in range(N + 1)
        ]

        if self.change_index:
            self.closest_index = np.argmin(
                np.sum((self.path_blf - [0, 0]) ** 2, axis=1)
            )
        else:
            distances_temp = np.sum((self.path_blf - [0, 0]) ** 2, axis=1)
            # Add a large distance to all indices 20 further than self.closest_idnex
            # To avoid mistake at skidpad overlap
            distances_temp = np.where(
                abs(np.arange(len(distances_temp)) - self.closest_index) < 20,
                distances_temp,
                distances_temp + 1000,
            )
            self.closest_index = np.argmin(distances_temp)
        if debug:
            current_point = self.path_blf[self.closest_index]
            print(current_point)

        # Now calculate for each point the distance to the car along the path

        # Shift path so that closest point is at index 0
        shifted_path = np.roll(self.path_blf, -self.closest_index, axis=0)
        # Compute the distance between consecutive points
        diff = np.diff(shifted_path, axis=0)
        distances_cumsum = np.linalg.norm(diff, axis=1)

        # Append 0 and calculate cummulative sum
        distances_relative = np.append([0], np.cumsum(distances_cumsum))

        # Find points at specified distances
        reference_path = []
        for i in range(len(distances)):
            # Find first value in distances_relative that is greater than distance
            for j in range(len(distances_relative)):
                if distances_relative[j] < distances[i]:
                    continue
                elif distances_relative[j] == distances[i]:
                    # Just take point on path
                    scaling = 1
                elif distances_relative[j] > distances[i]:
                    # Required distances between two points so scale between them
                    scaling = 1 - np.abs(
                        (distances[i] - distances_relative[j])
                        / (distances_relative[j] - distances_relative[j - 1])
                    )

                reference_path.append(
                    shifted_path[j - 1]
                    + scaling * (shifted_path[j] - shifted_path[j - 1])
                )
                if debug:
                    print(
                        f"Point {j - 1} and {j} with scaling {scaling} and distance {distances[i]} and relative distance {distances_relative[j]} results in point {reference_path[-1]}"
                    )
                break

        return reference_path

    def get_reference_path_gen(self, GT_centerline, dt, N, target_speed, debug=False):
        """
        Calculate the reference track based on the centerline and target speed
        Different from the above function, as this reference track does not depend on current speed
        """
        self.path_blf = GT_centerline

        if len(self.path_blf) == 0:
            return []

        # Find target points based on required velocity and maximum acceleration
        speed_target = target_speed
        # max_acceleration = 2.0  # TODO: create param
        # actual_speed = 0
        # calculate distances based on maximum acceleration and current speed
        distances = [((speed_target)) * i * dt for i in range(N + 1)]
        self.closest_index = np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
        self.closest_index = 0

        if debug:
            current_point = self.path_blf[self.closest_index]
            print(current_point)

        # Shift path so that closest point is at index 0
        shifted_path = np.roll(self.path_blf, -self.closest_index, axis=0)
        # Compute the distance between consecutive points
        diff = np.diff(shifted_path, axis=0)
        distances_cumsum = np.linalg.norm(diff, axis=1)

        # Append 0 and calculate cummulative sum
        distances_relative = np.append([0], np.cumsum(distances_cumsum))

        # Find points at specified distances

        reference_path = []
        for i in range(len(distances)):
            # Find first value in distances_relative that is greater than distance
            for j in range(len(distances_relative)):
                if distances_relative[j] < distances[i]:
                    continue
                elif distances_relative[j] == distances[i]:
                    # Just take point on path
                    scaling = 1
                elif distances_relative[j] > distances[i]:
                    # Required distances between two points so scale between them
                    scaling = 1 - np.abs(
                        (distances[i] - distances_relative[j])
                        / (distances_relative[j] - distances_relative[j - 1])
                    )

                reference_path.append(
                    shifted_path[j - 1]
                    + scaling * (shifted_path[j] - shifted_path[j - 1])
                )
                if debug:
                    print(
                        f"Point {j - 1} and {j} with scaling {scaling} and distance {distances[i]} and relative distance {distances_relative[j]} results in point {reference_path[-1]}"
                    )
                break
        # print(f"ref path 2: {reference_path}")

        # end_time = perf_counter()

        # print(f"Time 1: {mid_time - start_time}")
        # print(f"Time 2: {end_time - mid_time}")

        return reference_path
