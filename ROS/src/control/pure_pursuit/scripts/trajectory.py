import numpy as np
from scipy.interpolate import interp1d
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Header
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
import tf2_ros as tf
import rospy


class Trajectory:
    """
    Helper class to calculate the cars target point based on a given path it has to follow
    """

    def __init__(self):
        self.path = np.array([])
        self.eucl_dist = np.array([])
        self.closest_index = 0

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")


        self.frame_id = self.base_link_frame
        self.stamp = rospy.Time.now()
        

    def transform_blf(self, points):
        """
        transforms a path, given in points, to base_link_frame

        Args:
            points: (N, 2) numpy array
        
        """
        self.stamp = rospy.Time.now()
        trans = self.tf_buffer.lookup_transform(
        self.base_link_frame,
        self.frame_id,
        rospy.Time(0)
        )

        new_header = Header(frame_id=self.base_link_frame, stamp=rospy.Time.now())
        transformed_path = Path(header=new_header)

        poses = []
        for point in points:
            pose = PoseStamped(header = Header(frame_id = self.frame_id, stamp= self.stamp))

            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            pose_t = do_transform_pose(pose, trans)

            transformed_path.poses.append(pose_t)
        
        return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in transformed_path.poses])


    def set_path(self, points, current_position):
        """
        Sets the internal path with new points. Expects a numpy array of shape (N, 2)
        Path should be given in world frame

        Finds closest point on path and uses this as anker point.

        Args:
            points: (N,2) numpy array
            current_position: (x, y) of car
        """

        # Interpolate
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]
        alpha = np.linspace(0, 1, len(points) * 10)
        interpolator = interp1d(distance, points, kind="linear", axis=0)

        self.path = interpolator(alpha)
        self.path_blf = self.transform_blf(self.path)

        # Calculate closest point index
        self.eucl_dist = np.sqrt(
            (self.path_blf[:, 0] - current_position[0]) ** 2
            + (self.path_blf[:, 1] - current_position[1]) ** 2
        )
        # self.closest_index = np.argmin(self.eucl_dist)
        

    def calculate_target_point(
        self, minimal_distance, maximal_distance, current_position
    ):
        """
        Calculates a target point by traversing the path
        Returns the first points that matches the conditions given by minimal_distance

        Args:
            minimal_distance: minimal lookahead distance
            maximal_distance: maximal lookahead distance
            current_position: Current position of car

        Returns:
            x {float}: x position of target point
            y {float}: y position of target point
            success {bool}: True when target point was found
        """
        # transform path to most recent blf
        self.path_blf = self.transform_blf(self.path)

        # Iterate until found
        found = False
        # i = self.closest_index % len(self.path)

        while not found:
            distance = (current_position[0] - self.path_blf[self.closest_index][0]) ** 2 + (
                current_position[1] - self.path_blf[self.closest_index][1]
            ) ** 2
            if distance > maximal_distance**2:
                return (
                    self.path_blf[self.closest_index][0],
                    self.path_blf[self.closest_index][1],
                    False,
                )

            if distance > minimal_distance**2:
                return (
                    self.path_blf[self.closest_index][0],
                    self.path_blf[self.closest_index][1],
                    True,
                )

            self.closest_index = (self.closest_index + 1) % len(self.path_blf)
