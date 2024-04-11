#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from node_fixture import AutonomousMission, ROSNode
from scipy.interpolate import interp1d, splev, splprep


class PoseArraySmootherNode:
    def __init__(self):
        rospy.init_node("pose_array_smoother_node", anonymous=True)

        self.max_distance_away_from_start = rospy.get_param(
            "~max_distance_away_from_start", 9
        )
        self.min_distance_away_from_start = rospy.get_param(
            "~min_distance_away_from_start", 16
        )

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.final_path = False
        self.path_final = None
        # Subscriber and publisher
        self.subscriber = rospy.Subscriber(
            "/input/path", Path, self.pose_array_callback
        )
        self.publisher = rospy.Publisher("/output/path", Path, queue_size=10)

    def pose_array_callback(self, msg: Path):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, msg.header.stamp
            )

            transformed_msg = ROSNode.do_transform_path(msg, trans)
            transformed_msg.header.frame_id = self.world_frame
            mission = rospy.get_param("/mission", "")
            path = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in transformed_msg.poses]
            )

            # Find point closest to 0, 0
            closest_point = np.argmin(np.sum(path**2, axis=1))

            # Rotate path so that closest point is first
            path = np.roll(path, -closest_point, axis=0)
            away_from_start = False

            per = 0  # BSpline periodicity, 0 = not periodic, 1 = periodic

            # Determine loop closure and BSpline periodicity
            for node in path:
                distance_lc = (path[0, 0] - node[0]) ** 2 + (path[0, 1] - node[1]) ** 2
                if away_from_start and (
                    distance_lc < self.max_distance_away_from_start
                ):
                    per = 1
                    break
                if not away_from_start and (
                    distance_lc > self.min_distance_away_from_start
                ):
                    away_from_start = True

            # Add zero pose to path if no closure of path
            if not per and mission == AutonomousMission.TRACKDRIVE:
                path = np.vstack(([0, 0], path))
            # Linear interpolation between center points to add more points for BSpline smoothing
            distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
            distance = np.insert(distance, 0, 0) / distance[-1]

            alpha = np.linspace(0, 1, len(path) * 3)
            interpolator = interp1d(distance, path, kind="linear", axis=0)
            path = interpolator(alpha)

            # Smooth path with BSpline interpolation
            path = (
                path.T
            )  # Transpose to get correct shape for BSpline, splprep expects (2, N)
            w = np.array([1] * len(path[0]))  # Weights for BSpline
            # w[0] = 1
            # w[-1] = 1

            tck, u = splprep(path, w=w, s=10, per=per)  # Calculate BSpline
            smoothed_path = np.array(
                splev(u, tck)
            ).T  # Evaluate BSpline and transpose back to (N, 2)

            smoothed_msg = transformed_msg
            smoothed_msg.poses = []
            for point in smoothed_path:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                smoothed_msg.poses.append(pose)

            if per and not self.final_path:
                self.final_path = True
                self.path_final = smoothed_msg
                print("Final path found")
                print("Final path found")
                print("Final path found")
                print("Final path found")
                print("Final path found")

            # if self.final_path:
            #     self.publisher.publish(self.path_final)
            # else:
            self.publisher.publish(smoothed_msg)
        except Exception as e:
            rospy.logerr("Error occurred while smoothing PoseArray: {}".format(e))
            self.publisher.publish(msg)


if __name__ == "__main__":
    try:
        node = PoseArraySmootherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
