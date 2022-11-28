import numpy as np
from sensor_msgs.msg import Image
import rospy

def np_to_ros_image(arr: np.ndarray) -> Image:
    """Creates a ROS image type based on a Numpy array
    Args:
        arr: numpy array in RGB format (H, W, 3), datatype uint8
    Returns:
        ROS Image with appropriate header and data
    """

    ros_img = Image(encoding="rgb8")
    ros_img.height, ros_img.width, _ = arr.shape
    contig = arr  # np.ascontiguousarray(arr)
    ros_img.data = contig.tobytes()
    ros_img.step = contig.strides[0]
    ros_img.is_bigendian = (
        arr.dtype.byteorder == ">"
        or arr.dtype.byteorder == "="
        and sys.byteorder == "big"
    )

    ros_img.header.stamp = rospy.Time.now()
    ros_img.header.frame_id = "ugr/car_base_link/sensors/cam0"

    return ros_img


def ros_img_to_np(image: Image) -> np.ndarray:
    """Converts a ros image into an numpy array
    Args:
        image: ros image
    returns:
        numpy array containing the image data

    """
    im = np.frombuffer(image.data, dtype=np.uint8).reshape(
        image.height, image.width, -1
    )

    return im
