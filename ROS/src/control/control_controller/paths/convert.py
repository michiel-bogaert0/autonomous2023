import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
import yaml
from genpy.message import fill_message_args
from io import BytesIO

def convert_posearray_to_path(posearray_msg):
    path_msg = Path()
    path_msg.header = posearray_msg.header

    for pose in posearray_msg.poses:
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header = posearray_msg.header
        path_msg.poses.append(pose_stamped)

    return path_msg

ros_path = PoseArray()

filename = "./straight_L100.yaml"

with open(filename, "r") as file:
    path = yaml.safe_load(file)
    fill_message_args(ros_path, path) 

    converted_path = convert_posearray_to_path(ros_path)

with open(filename, "w") as file:
    file.write(str(converted_path))
