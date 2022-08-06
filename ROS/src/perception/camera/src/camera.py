#! /usr/bin/python3.8

from publisher_abstract.publisher_abstract import PublishNode
import cv2 as cv
import rospy


class CameraNode(PublishNode):
    def __init__():
        super().__init__("camera")

    def setup_camera(self) -> None:
        """Sets up the Baumer camera with the right settings
        Returns:
            Sets the self.camera object
        """

        # Init camera
        camera = neoapi.Cam()
        camera.Connect("700006709126")

        # Get the configuration saved on the camera itself
        selector = camera.GetFeature("UserSetSelector")
        selector.value = "UserSet1"

        user = camera.GetFeature("UserSetLoad")
        user.Execute()

        # Use continuous mode
        camera.f.TriggerMode.value = (
            neoapi.TriggerMode_Off
        )  # set camera to trigger mode, the camera starts streaming
        camera.f.AcquisitionFrameRateEnable = (
            True  # enable the frame rate control (optional)
        )
        camera.f.AcquisitionFrameRate = 24  # set the frame rate to 24 fps (optional)

        if camera.f.PixelFormat.GetEnumValueList().IsReadable("BGR8"):
            camera.f.PixelFormat.SetString("BGR8")

        # Limit the height of the output image
        height = rospy.get_param("~heigth",900)
        camera.f.Height = height
        camera.f.OffsetY = 1200 - height

        self.camera = camera

    def np_to_ros_image(self, arr: np.ndarray) -> Image:
        """Creates a ROS image type based on a Numpy array
        Args:
            arr: numpy array in RGB format (H, W, 3), datatype uint8
        Returns:
            ROS Image with appropriate header and data
        """

        ros_img = Image(encoding="rgb8")
        ros_img.height, ros_img.width, _ = arr.shape
        contig = arr  # np.ascontiguousarray(arr)
        ros_img.data = contig.tostring()
        ros_img.step = contig.strides[0]
        ros_img.is_bigendian = (
            arr.dtype.byteorder == ">"
            or arr.dtype.byteorder == "="
            and sys.byteorder == "big"
        )

        ros_img.header.stamp = rospy.Time.now()
        ros_img.header.frame_id = "ugr/car_base_link/sensors/cam0"

        return ros_img

    def process_data(self):
        """
        reads the frames from the mp4 file

        returns:
            Ros Image to be published
        """

        image = camera.GetImage()

        if not image.IsEmpty():
            img = image.GetNPArray()

            ros_img = self.np_to_ros_image(image.GetNPArray())

            return ros_img


node = CameraNode()
node.publish_image_data()
