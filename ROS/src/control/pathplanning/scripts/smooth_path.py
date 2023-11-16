#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.interpolate import interp1d, splev, splprep


class PoseArraySmootherNode:
    def __init__(self):
        rospy.init_node("pose_array_smoother_node", anonymous=True)

        # Subscriber and publisher
        self.subscriber = rospy.Subscriber(
            "/input/path", Path, self.pose_array_callback
        )
        self.publisher = rospy.Publisher("/output/path", Path, queue_size=10)

    def pose_array_callback(self, msg: Path):
        try:
            path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
            distance = np.insert(distance, 0, 0) / distance[-1]
            alpha = np.linspace(0, 1, len(path) * 3)
            interpolator = interp1d(distance, path, kind="linear", axis=0)
            path = interpolator(alpha)

            alpha = np.linspace(0, len(path), 1)
            path = path.T
            w = np.array([1] * len(path[0]))

            # m = len(path[0])
            # s = m - (2 * m) ** (1 / 2)
            tck, u = splprep(path, w=w, s=10)
            print(tck)
            smoothed_path = np.array(splev(u, tck)).T

            smoothed_msg = msg
            smoothed_msg.poses = []
            for point in smoothed_path:
                pose = PoseStamped()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                smoothed_msg.poses.append(pose)

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
