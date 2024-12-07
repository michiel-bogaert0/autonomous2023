#!/usr/bin/env python3

import os

import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header


class VideoNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("video_analyser", anonymous=True)

        # Parameters
        self.video_path = rospy.get_param(
            "~video_path", f"{os.path.dirname(__file__)}/../video.mov"
        )
        self.topic_name = rospy.get_param("~output_topic", "/video_frames/compressed")
        self.frame_rate = rospy.get_param("~frame_rate", 30)

        # Publisher for CompressedImage
        self.image_pub = rospy.Publisher(
            self.topic_name, CompressedImage, queue_size=10
        )

        # Video processing
        self.process_video()

    def process_video(self):
        # Open the video file
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            rospy.logerr(f"Cannot open video file {self.video_path}")
            return

        rate = rospy.Rate(self.frame_rate)

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.loginfo("End of video stream or error reading the video")
                break

            try:
                # Compress the frame to JPEG format
                msg = CompressedImage()
                msg.header = Header(stamp=rospy.Time.now())
                msg.format = "jpeg"  # Format can be "jpeg" or "png"

                # Encode the frame as JPEG with quality 80 (adjustable)
                _, encoded_frame = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                msg.data = encoded_frame.tobytes()

                # Publish the compressed message
                self.image_pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Error converting/publishing compressed frame: {e}")

            rate.sleep()

        cap.release()


if __name__ == "__main__":
    try:
        n = VideoNode()
    except rospy.ROSInterruptException:
        pass
