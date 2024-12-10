#!/usr/bin/env python3

import os

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header


class VideoNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("video_analyser", anonymous=True)

        # Parameters
        self.video_path = rospy.get_param(
            "~video_path", f"{os.path.dirname(__file__)}/../video.mov"
        )
        self.compressed_topic = rospy.get_param(
            "~compressed_topic", "/video_frames/compressed"
        )
        self.normal_topic = rospy.get_param("~normal_topic", "/video_frames/normal")
        self.frame_rate = rospy.get_param("~frame_rate", 30)

        # Publishers
        self.compressed_pub = rospy.Publisher(
            self.compressed_topic, CompressedImage, queue_size=10
        )
        self.normal_pub = rospy.Publisher(self.normal_topic, Image, queue_size=10)

        # Video processing
        self.bridge = CvBridge()  # OpenCV to ROS bridge
        self.process_video()

    def process_video(self):
        while not rospy.is_shutdown():
            # Open the video file
            cap = cv2.VideoCapture(self.video_path)
            if not cap.isOpened():
                rospy.logerr(f"Cannot open video file {self.video_path}")
                return

            rospy.loginfo(f"Playing video {self.video_path}...")
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    rospy.loginfo("Reached end of video. Restarting...")
                    break  # Restart the video

                try:
                    # Publish compressed frame
                    compressed_msg = CompressedImage()
                    compressed_msg.header = Header(stamp=rospy.Time.now())
                    compressed_msg.format = "jpeg"
                    _, encoded_frame = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                    )
                    compressed_msg.data = encoded_frame.tobytes()
                    self.compressed_pub.publish(compressed_msg)

                    # Convert to RGB for the normal Image message
                    rgb_frame = cv2.cvtColor(
                        frame, cv2.COLOR_BGR2RGB
                    )  # Convert BGR to RGB
                    normal_msg = self.bridge.cv2_to_imgmsg(
                        rgb_frame, encoding="rgb8"
                    )  # Specify rgb8 encoding
                    normal_msg.header = Header(stamp=rospy.Time.now())
                    self.normal_pub.publish(normal_msg)
                except Exception as e:
                    rospy.logerr(f"Error converting/publishing frames: {e}")

                rospy.Rate(self.frame_rate).sleep()

            # Release the video capture before restarting
            cap.release()


if __name__ == "__main__":
    try:
        n = VideoNode()
    except rospy.ROSInterruptException:
        pass
