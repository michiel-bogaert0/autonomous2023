import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow
    """

    def __init__(self):
        self.closest_index = 0
        self.points = np.array([])
        self.target = np.array([0, 0])

        # change indix is True for trackdrive/autocross, false for skidpad/acc
        self.change_index = rospy.get_param("~change_index", True)
        rospy.loginfo(f"change: {self.change_index} ")
        # Transformations
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        # for skidpad/acc use ugr/map, for trackdrive/autocross use ugr/car_odom
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        # rospy.loginfo(f"world_frame: {self.world_frame} ")
        # rospy.loginfo(f"world_frame: {self.world_frame} ")
        # rospy.loginfo(f"world_frame: {self.world_frame} ")
        # rospy.loginfo(f"world_frame: {self.world_frame} ")
        # rospy.loginfo(f"world_frame: {self.world_frame} ")
        self.time_source = rospy.Time(0)

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

        # Transform
        new_header = Header(
            frame_id=self.base_link_frame, stamp=self.trans.header.stamp
        )
        transformed_path = Path(header=new_header)

        for point in self.points:
            pose = PoseStamped(
                header=Header(
                    frame_id=self.base_link_frame, stamp=self.trans.header.stamp
                )
            )

            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            pose_t = do_transform_pose(pose, self.trans)

            transformed_path.poses.append(pose_t)

        # save points for next transform
        self.points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in transformed_path.poses
            ]
        )
        return self.points

    def calculate_target_point(self, minimal_distance):
        """
        Calculates a target point by traversing the path
        Returns the first points that matches the conditions given by minimal_distance

        Args:
            minimal_distance: minimal lookahead distance

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
            success {bool}: True when target point was found
        """
        # transform path to most recent blf
        self.path_blf = self.transform_blf()

        # Only calculate closest index as index of point with smallest distance to current position if working in trakdrive/autocross
        current_position_index = (
            np.argmin(np.sum((self.path_blf - [0, 0]) ** 2, axis=1))
            if self.change_index
            else self.closest_index
        )
        self.closest_index = current_position_index

        # Iterate until found
        found = False
        while not found:
            target_x = self.path_blf[self.closest_index][0]
            target_y = self.path_blf[self.closest_index][1]

            # current position is [0,0] in base_link_frame
            distance = (0 - target_x) ** 2 + (0 - target_y) ** 2

            if distance > minimal_distance**2:
                self.target = np.array([target_x, target_y])
                return (self.target[0], self.target[1])

            self.closest_index = (self.closest_index + 1) % len(self.path_blf)

            # for trackdrive/autocross, return latest target point if no point is found further away than minimal_distance
            if self.closest_index == current_position_index:
                pose = PoseStamped(
                    header=Header(
                        frame_id=self.base_link_frame, stamp=self.trans.header.stamp
                    )
                )

                pose.pose.position.x = self.target[0]
                pose.pose.position.y = self.target[1]

                pose_t = do_transform_pose(pose, self.trans)

                self.target = np.array([pose_t.pose.position.x, pose_t.pose.position.y])
                return (self.target[0], self.target[1])
