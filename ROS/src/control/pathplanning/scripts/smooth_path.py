#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.interpolate import interp1d, make_interp_spline


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

            alpha = np.linspace(0, 1, len(path))
            bspline = make_interp_spline(alpha, path, k=2)

            alpha_new = np.linspace(0, 1, len(path) * 20)
            smoothed_path = bspline(alpha_new)

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
