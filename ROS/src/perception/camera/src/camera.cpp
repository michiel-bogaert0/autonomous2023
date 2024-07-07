#include <camera.hpp>
#include <iostream>

CameraNode::CameraNode(ros::NodeHandle &n) {
  /* Getting ROS parameters */
  n.param<bool>("use_raw", use_raw_, false);
  n.param<bool>("cam_rotated", cam_rotated_, false);
  n.param<std::string>("camsettings", camsettings_location_,
                       "camera_settings.yaml");
  n.param<double>("estimated_latency", estimated_latency_, 0.27);
  n.param<int>("rate", rate_value_, 10);
  n.param<std::string>("sensor_name", sensor_name_, "cam0");
  n.param<std::string>("camcal_location", camcal_location_,
                       "camera_calibration_baumer.npz");

  /* Setting up ROS publishers/subscribers */
  raw_sub_ = n.subscribe("/raw/input", 10, &CameraNode::publish_sub_data, this);
  image_pub_ = n.advertise<sensor_msgs::Image>("/input/image", 10);
  info_pub_ = n.advertise<sensor_msgs::CameraInfo>("/input/info", 10);

  /* Creating ROS rate object */
  rate_ = ros::Rate(rate_value_);

  /* Setting up frame variable */
  frame_ = "ugr/car_base_link/";
  frame_.append(sensor_name_);

  /* Reading camera settings yaml file */
  std::string path_to_camsettings_ = std::getenv("BINARY_LOCATION");
  path_to_camsettings_.append("/pnp/");
  path_to_camsettings_.append(camsettings_location_);
  camsettings_ = YAML::LoadFile(path_to_camsettings_);
  camera_height_ = camsettings_["height"].as<int>();
  camera_width_ = camsettings_["width"].as<int>();

  setup_camera();
  publish_image_data();
};

std_msgs::Header CameraNode::create_header() {
  /**
   * @brief Returns a header containing the frame and current timestamp
   * (adjusted for the estimated latency). Both the frame and latency are
   * determined by their ROS parameters.
   */
  std_msgs::Header header = std_msgs::Header();
  header.stamp = ros::Time::now() - ros::Duration(estimated_latency_);
  header.frame_id = frame_;
  return header;
};

sensor_msgs::Image CameraNode::cv_to_ros_image(cv::Mat &mat) {
  /**
   * @brief Takes a cv::Mat matrix and reinterprets it as a ROS image message.
   */
  cv_bridge::CvImage msg;
  msg.header = create_header();
  msg.encoding = sensor_msgs::image_encodings::RGB8;
  msg.image = mat;

  return *msg.toImageMsg();
};

void CameraNode::setup_camera() {
  /**
   * @brief Sets up the Baumer camera with the right settings.
   */

  /* Init Camera */
  camera_ = NeoAPI::Cam();
  camera_.Connect("700006709126");

  /* Get the configuration saved on the camera itself */
  NeoAPI::Feature selector = camera_.GetFeature("UserSetSelector");
  selector.SetString("UserSet1");

  NeoAPI::Feature user = camera_.GetFeature("UserSetLoad");
  user.Execute();

  /* Use continuous mode */
  camera_.f().TriggerMode.Set(
      NeoAPI::TriggerMode::Off); /* set camera to trigger mode, the camera
                                    starts streaming */
  camera_.f().AcquisitionFrameRateEnable.Set(
      true); /* enable the frame rate control (optional) */

  camera_.f().AcquisitionFrameRate.Set(
      24); /* set the frame rate to 24 fps (optional) */

  if (camera_.f().PixelFormat.GetEnumValueList().IsReadable("BGR8")) {
    camera_.f().PixelFormat.SetString("BGR8");
  }
  /* Calibration parameters */

  *camera_matrix_ =
      (cv::Mat_<double>(3, 3) << 1.66170824e+03, 0, 9.59900862e+02, 0,
       1.65718865e+03, 5.98103114e+02, 0, 0, 1);
  *distortion_matrix_ =
      (cv::Mat_<double>(1, 5) << -1.08573207e-01, 1.74070981e-01,
       -6.25355801e-05, 1.55457374e-03, -1.01938235e-01);
  *optimal_camera_matrix_ = cv::getOptimalNewCameraMatrix(
      *camera_matrix_, cv::Mat::zeros(1, 4, CV_64FC1),
      cv::Size(camera_width_, camera_height_), 1);
  cv::initUndistortRectifyMap(
      *camera_matrix_, *distortion_matrix_, cv::Mat(), *optimal_camera_matrix_,
      cv::Size(camera_width_, camera_height_), CV_32FC1, *map1_, *map2_);
};

sensor_msgs::Image CameraNode::process_data() {
  /**
   * @brief Gets image from the Baumer camera, reinterprets it as a cv::Mat, and
   * undistorts the image. Returns a ROS image message.
   */
  NeoAPI::Image image = camera_.GetImage();

  if (!image.IsEmpty()) {
    void *buf;
    buf = image.GetImageData();
    cv::Mat img(cv::Size(camera_width_, camera_height_), CV_8UC3, buf,
                cv::Mat::AUTO_STEP);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    if (cam_rotated_) {
      cv::rotate(img, img, cv::ROTATE_180); // Rotate the image by 180 degrees
    }
    if (!use_raw_)
      cv::remap(img, img, *map1_, *map2_, cv::INTER_LINEAR);
    sensor_msgs::Image msg = cv_to_ros_image(img);
    return msg;
  } else {
    cv::Mat img(cv::Size(camera_width_, camera_height_), CV_8UC3);
    sensor_msgs::Image msg = cv_to_ros_image(img);
    return msg;
  }
};

sensor_msgs::CameraInfo CameraNode::get_camera_info() {
  /**
   * @brief Copies the camera info to a CameraInfo ROS message.
   */
  sensor_msgs::CameraInfo msg = sensor_msgs::CameraInfo();
  msg.height = camera_height_;
  msg.width = camera_width_;

  if (use_raw_) {
    cv::Mat flat_dist = distortion_matrix_->reshape(1);
    std::vector<double> dist(flat_dist.data,
                             flat_dist.data + flat_dist.total());
    msg.D = dist;
    msg.K = mat_to_boost<9>(*camera_matrix_);
    cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_64FC1);
    msg.R = mat_to_boost<9>(eye_mat);
    cv::Mat stacked_camera_matrix_;
    cv::hconcat(*optimal_camera_matrix_, cv::Mat::zeros(3, 1, CV_64F),
                stacked_camera_matrix_);
    msg.P = mat_to_boost<12>(stacked_camera_matrix_);
  } else {
    cv::Mat flat_zeros = cv::Mat::zeros(1, 5, CV_64FC1);
    std::vector<double> vec_zeros(flat_zeros.data,
                                  flat_zeros.data + flat_zeros.total());
    msg.D = vec_zeros;
    msg.K = mat_to_boost<9>(*optimal_camera_matrix_);
    cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_64FC1);
    msg.R = mat_to_boost<9>(eye_mat);
    cv::Mat stacked_camera_matrix_;
    cv::hconcat(*optimal_camera_matrix_, cv::Mat::zeros(3, 1, CV_64F),
                stacked_camera_matrix_);
    msg.P = mat_to_boost<12>(stacked_camera_matrix_);
  };
  return msg;
};

void CameraNode::publish_sub_data(const sensor_msgs::Image &msg) {
  /**
   * @brief Wrapper for fsds sim.
   */
  image_pub_.publish(msg);
};

void CameraNode::publish_image_data() {
  /**
   * @brief The main loop for the camera node. Constantly fetches images from
   * the camera using the process_data method, and publishes. Also publishes
   * CameraInfo messages.
   */
  while (ros::ok()) {
    sensor_msgs::Image data = process_data();
    data.header = create_header();
    image_pub_.publish(data);

    sensor_msgs::CameraInfo info = get_camera_info();
    info.header = data.header;
    info_pub_.publish(info);

    rate_.sleep();
  }
};

template <int Size>
boost::array<double, Size> CameraNode::mat_to_boost(cv::Mat &mat) {
  /**
   * @brief Reinterprets cv::Mat object into a boost array of a certain size.
   * Needed for the get_camera_info method.
   */
  cv::Mat flat = mat.reshape(1, mat.total() * mat.channels());
  std::vector<double> vec = mat.isContinuous() ? flat : flat.clone();
  boost::array<double, Size> arr;
  boost::range::copy(vec, arr.begin());
  return arr;
};
