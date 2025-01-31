#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rrt.rrt import Rrt
from scipy.interpolate import interp1d
from std_msgs.msg import Header
from triangulation.triangulator import Triangulator
from ugr_msgs.msg import ObservationWithCovarianceArrayStamped
from visualization_msgs.msg import MarkerArray


class PathPlanning:
    """Path planning node. Calculates and publishes path based on observations."""

    def __init__(self) -> None:
        """Initialize node"""
        rospy.init_node("exploration_mapping")

        self.frametf = TransformFrames()

        self.params = {}

        # Do we want to output visualisations for diagnostics?
        self.params["debug_visualisation"] = rospy.get_param(
            "~debug_visualisation", False
        )
        self.params["vis_namespace"] = rospy.get_param(
            "~vis_namespace", "pathplanning_vis"
        )
        self.params["vis_lifetime"] = rospy.get_param("~vis_lifetime", 0.2)

        # Defines which algorithm to run triangulatie ("tri") or RRT ("RRT")
        self.params["algo"] = rospy.get_param("~algorithm", "tri")

        # Load at least all params from config file via ros parameters

        # General parameters
        # The amount of branches generated
        self.params["max_iter"] = rospy.get_param("~max_iter", 1000)
        # Early prune settings
        # The maximum angle (rad) for a branch to be valid (sharper turns will be pruned prematurely)
        self.params["max_angle_change"] = rospy.get_param("~max_angle_change", 0.5)
        # The radius around a obstacle (cone) where no path can be planned
        # Should be at least the half the width of the car
        self.params["safety_dist"] = rospy.get_param("~safety_dist", 1)

        # Extra parameters for triangulation
        # The minimal variance each allowed set of triangle edge lengths can always have.
        #   So it's the minimal maximum variance
        self.params["triangulation_max_var"] = rospy.get_param(
            "~triangulation_max_var", 200
        )
        # Factor multiplied to the median of the variance of triangle lengths in order to filter bad triangles
        self.params["triangulation_var_threshold"] = rospy.get_param(
            "~triangulation_var_threshold", 1.2
        )
        # Maximum distance between nodes in the planned path (paths with nodes further than this will be pruned prematurely)
        self.params["max_path_distance"] = rospy.get_param("~max_path_distance", 6)

        # continuous_dist: the max distance between nodes in stage 1
        # stage1_rect_width: width of the rectangle for line expansion
        # stage1_bad_points_threshold: threshold for the amount of bad points crossings allowed
        # stage1_center_points_threshold: threshold for the amount of center point (not used) crossings allowed
        # stage2_rect_width: width of the rectangle for line expansion
        # stage2_bad_points_threshold: threshold for the amount of bad points crossings allowed
        # stage2_center_points_threshold: threshold for the amount of center point (not used) crossings allowed
        # max_depth: maximal depth for path searching in stage 2
        self.params["continuous_dist"] = rospy.get_param("~continuous_dist", 4)
        self.params["stage1_rectangle_width"] = rospy.get_param(
            "~stage1_rectangle_width", 1.2
        )
        self.params["stage1_bad_points_threshold"] = rospy.get_param(
            "~stage1_bad_points_threshold", 1
        )
        self.params["stage1_center_points_threshold"] = rospy.get_param(
            "~stage1_center_points_threshold", 3
        )
        self.params["stage2_rectangle_width"] = rospy.get_param(
            "~stage2_rectangle_width", 1.2
        )
        self.params["stage2_bad_points_threshold"] = rospy.get_param(
            "~stage2_bad_points_threshold", 1
        )
        self.params["stage2_center_points_threshold"] = rospy.get_param(
            "~stage2_center_points_threshold", 2
        )
        self.params["max_depth"] = rospy.get_param("~max_depth", 5)

        # The range in front of the car where cones should be kept
        self.params["range_front"] = rospy.get_param("~range_front", 10)
        # The range behind the car where cones should be kept
        self.params["range_behind"] = rospy.get_param("~range_behind", 0)
        # The range to the sides of the car where cones should be kept
        self.params["range_sides"] = rospy.get_param("~range_sides", 3)

        # Extra parameters for RRT
        # The distance by which the car drives every update
        self.params["expand_dist"] = rospy.get_param("~expand_dist", 0.5)
        # The distance the car can see in front
        self.params["plan_dist"] = rospy.get_param("~plan_dist", 12.0)
        # Minimum or average width of the track
        # Used to estimate middle one side of cones is missing.
        self.params["track_width"] = rospy.get_param("~track_width", 3)
        # Maximum width of the track used to detect if RRT node is possibly out of the track
        self.params["max_track_width"] = rospy.get_param("~max_track_width", 4)
        # Used for RRT* variant to define radius to optimize new RRT node
        # When set to None, will be twice max_dist (expand_dist*3)
        self.params["search_rad"] = (
            None
            if rospy.get_param("~search_rad", None) is None
            or rospy.get_param("~search_rad", None) == "None"
            else rospy.get_param("~search_rad", None)
        )
        # Iteration threshold which triggers parameter update. (3/4 of max_iter seems to be ok)
        self.params["iter_threshold"] = rospy.get_param("~iter_threshold", 560)
        # Percentage to increase maximum angle when parameter update is triggered.
        self.params["angle_inc"] = rospy.get_param("~angle_inc", 0.2)
        # Factor in to increase maximum angle to create more chance for edges.
        self.params["angle_fac"] = rospy.get_param("~angle_fac", 1.5)

        # Create debug topics if needed
        self.vis_points = None
        self.vis_lines = None
        if self.params["debug_visualisation"]:
            self.vis_points = rospy.Publisher(
                "/output/debug/markers", MarkerArray, queue_size=10
            )
            self.vis_lines = rospy.Publisher(
                "/output/debug/poses", PoseArray, queue_size=10
            )

        if self.params["algo"] == "rrt":
            self.algorithm = Rrt(
                self.params["expand_dist"] * 3,
                self.params["plan_dist"],
                self.params["max_iter"],
                self.params["max_angle_change"],
                self.params["safety_dist"],
                self.params["track_width"],
                self.params["max_track_width"],
                self.params["search_rad"],
                self.params["iter_threshold"],
                self.params["angle_inc"],
                self.params["angle_fac"],
            )
        else:
            self.algorithm = Triangulator(
                self.params["triangulation_max_var"],
                self.params["triangulation_var_threshold"],
                self.params["max_iter"],
                self.params["max_angle_change"],
                self.params["max_path_distance"],
                self.params["safety_dist"],
                self.params["continuous_dist"],
                self.params["stage1_rectangle_width"],
                self.params["stage1_bad_points_threshold"],
                self.params["stage1_center_points_threshold"],
                self.params["stage2_rectangle_width"],
                self.params["stage2_bad_points_threshold"],
                self.params["stage2_center_points_threshold"],
                self.params["max_depth"],
                self.params["range_front"],
                self.params["range_behind"],
                self.params["range_sides"],
                vis_points=self.vis_points,
                vis_lines=self.vis_lines,
                vis_namespace=self.params["vis_namespace"],
                vis_lifetime=self.params["vis_lifetime"],
            )

        self.pub = rospy.Publisher("/output/path", PoseArray, queue_size=10)
        self.pub_stamped = rospy.Publisher("/output/path_stamped", Path, queue_size=10)

        rospy.Subscriber(
            "/input/local_map",
            ObservationWithCovarianceArrayStamped,
            self.receive_new_map,
        )
        rospy.spin()

    def receive_new_map(self, track: ObservationWithCovarianceArrayStamped):
        """Receives observations from input topic.

        Args:
            track: The observations/message on input topic.
        """
        cones = np.array(
            [
                [
                    obs_with_cov.observation.location.x,
                    obs_with_cov.observation.location.y,
                    obs_with_cov.observation.observation_class,
                ]
                for obs_with_cov in track.observations
            ]
        )

        # Compute
        self.compute(cones, track.header)

    def compute(self, cones: np.ndarray, header: Header) -> None:
        """Calculate path and publish it.

        Args:
            cones: array of cones (x, y, colour)
            header: Header of input message.
        """
        path = self.algorithm.get_path(cones, header)

        if path is None or len(path) == 0:
            rospy.loginfo("No path found")
            return

        path = np.array([[0, 0]] + [[e.x, e.y] for e in path])

        # Smooth path
        distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]
        alpha = np.linspace(0, 1, len(path) * 4)
        interpolator = interp1d(distance, path, kind="cubic", axis=0)

        path = interpolator(alpha)

        poses: list(Pose) = []
        for point in path:
            pose: Pose = Pose()
            position: Point = Point()
            orientation: Quaternion = Quaternion()

            # Fill position from path point
            position.x = point[0]
            position.y = point[1]
            position.z = 0

            # Fill orientation, to invalidate
            orientation.x = 0
            orientation.y = 0
            orientation.z = 0
            orientation.w = 0

            # Fill pose and add to array
            pose.position = position
            pose.orientation = orientation
            poses += [pose]

        output: PoseArray = PoseArray()
        output.header.frame_id = header.frame_id
        output.poses = poses
        output.header.stamp = header.stamp

        output_transformed = self.frametf.pose_transform(output)

        self.pub.publish(output_transformed)

        poses_stamped: list(PoseStamped) = []
        for pose in output_transformed.poses:
            posestamped: PoseStamped = PoseStamped(pose=pose)
            posestamped.header.frame_id = output_transformed.header.frame_id
            posestamped.header.stamp = output_transformed.header.stamp

            poses_stamped += [posestamped]

        stamped_output: Path = Path(
            header=output_transformed.header, poses=poses_stamped
        )

        self.pub_stamped.publish(stamped_output)


# Source: https://gitlab.msu.edu/av/av_notes/-/blob/master/ROS/Coordinate_Transforms.md
class TransformFrames:
    def __init__(self):
        """Create a buffer of transforms and update it with TransformListener."""
        self.tfBuffer = tf2_ros.Buffer()  # Creates a frame buffer
        tf2_ros.TransformListener(
            self.tfBuffer
        )  # TransformListener fills the buffer as background task

    def get_transform(self, source_frame, target_frame):
        """Lookup latest transform between source_frame and target_frame from the buffer."""
        try:
            trans = self.tfBuffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logerr(
                f"Cannot find transformation from {source_frame} to {target_frame}"
            )
            raise Exception(
                f"Cannot find transformation from {source_frame} to {target_frame}"
            ) from exc
        return trans  # Type: TransformStamped

    def pose_transform(self, pose_array: PoseArray) -> PoseArray:
        """Transform PoseArray to other frame.

        Args:
            pose_array: will be transformed to target_frame
        """
        target_frame = rospy.get_param("~output_frame", "odom")
        trans = self.get_transform(pose_array.header.frame_id, target_frame)
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp)
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            pose_array_transformed.poses.append(pose_t.pose)
        return pose_array_transformed

    def get_frame_A_origin_frame_B(self, frame_A, frame_B):
        """Returns the pose of the origin of frame_A in frame_B as a PoseStamped."""
        header = Header(frame_id=frame_A, stamp=rospy.Time(0))
        origin_A = Pose(
            position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        origin_A_stamped = PoseStamped(pose=origin_A, header=header)
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(
            origin_A_stamped, self.get_transform(frame_A, frame_B)
        )
        return pose_frame_B


node = PathPlanning()
