cmake_minimum_required(VERSION 3.0.2)
project(ugr_msgs)


find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    fs_msgs
    nav_msgs
)

add_message_files(
  FILES
  CarPath.msg
  Particle.msg
  Particles.msg
  Observation.msg
  ObservationArray.msg
  BoundingBox.msg
  BoundingBoxesStamped.msg
  Map.msg
  MapUpdate.msg
  ExplorationMove.msg
  State.msg
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
  PathArray.msg
  TrajectoryError.msg
  TrajectoryInfo.msg
  TrajectoryMapInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  fs_msgs
  nav_msgs
)

catkin_package(

)

include_directories(

)