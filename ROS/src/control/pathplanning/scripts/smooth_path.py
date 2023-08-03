#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseArray
from scipy.interpolate import interp1d
import numpy as np

class PoseArraySmootherNode:
    def __init__(self):
        rospy.init_node('pose_array_smoother_node', anonymous=True)

        # Subscriber and publisher
        self.subscriber = rospy.Subscriber('/input/path', PoseArray, self.pose_array_callback)
        self.publisher = rospy.Publisher('/output/path', PoseArray, queue_size=10)

    def pose_array_callback(self, msg: PoseArray):
        try:
            
            path = np.array([[p.position.x, p.position.y] for p in msg.poses])

            distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )) )
            distance = np.insert(distance, 0, 0)/distance[-1]
            alpha = np.linspace(0, 1, len(path) * 10)
            interpolator =  interp1d(distance, path, kind='quadratic', axis=0)
            
            path = interpolator(alpha)

            smoothed_msg = msg
            smoothed_msg.poses = []
            for point in path:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                smoothed_msg.poses.append(pose)

            self.publisher.publish(smoothed_msg)
        except Exception as e:
            rospy.logerr("Error occurred while smoothing PoseArray: {}".format(e))
            self.publisher.publish(msg)

if __name__ == '__main__':
    try:
        node = PoseArraySmootherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
