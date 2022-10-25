cmake_minimum_required(VERSION 3.0.2)
project(ugr_msgs)


find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    fs_msgs
)

add_message_files(
  FILES
  ConeKeypoint.msg
  CarPath.msg
  Particle.msg
  Particles.msg
  ConeKeypoints.msg
  Observation.msg
  ObservationArray.msg
  ConeDetections.msg
  BoundingBox.msg
  Map.msg
  MapUpdate.msg
  ExplorationMove.msg
  ExplorationPath.msg
  StateUpdate.msg
  Edge.msg
  VisExploration.msg
  ConeLocation.msg
  ObservationWithCovariance.msg
  ObservationStamped.msg
  ObservationWithCovarianceStamped.msg
  ObservationWithCovarianceArray.msg
  ObservationWithCovarianceArrayStamped.msg
  ObservationArrayWithCovarianceStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  fs_msgs
)

catkin_package(

)

include_directories(

)