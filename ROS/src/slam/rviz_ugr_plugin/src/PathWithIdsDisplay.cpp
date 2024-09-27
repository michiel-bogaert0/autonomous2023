#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <rviz/display.h>
#include <rviz/ogre_helpers/shape.h>

#include "PathWithIdsDisplay.hpp"

namespace rviz_path_with_ids {
PathWithIdsDisplay::PathWithIdsDisplay() {}

void PathWithIdsDisplay::onInitialize() { MFDClass::onInitialize(); }

PathWithIdsDisplay::~PathWithIdsDisplay() {}

// Clear the visuals by deleting their objects.
void PathWithIdsDisplay::reset() {
  MFDClass::reset();
  clearDisplay();
}

// This is our callback to handle an incoming message.
void PathWithIdsDisplay::processMessage(
    const ugr_msgs::PathWithIds::ConstPtr &msg) {

  // Clear any previous visualization
  clearDisplay();

  // Get transformation
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Loop through the poses in the path message
  for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
    const ugr_msgs::PoseStampedWithIds &pose1 = msg->poses[i];
    const ugr_msgs::PoseStampedWithIds &pose2 = msg->poses[i + 1];
    // Create a line segment between consecutive poses
    rviz::Line *line = new rviz::Line(context_->getSceneManager(), scene_node_);
    Ogre::Vector3 point1(pose1.pose.position.x, pose1.pose.position.y,
                         pose1.pose.position.z);
    Ogre::Vector3 point2(pose2.pose.position.x, pose2.pose.position.y,
                         pose2.pose.position.z);

    point1 = (orientation * point1) + position;
    point2 = (orientation * point2) + position;

    line->setPoints(point1, point2);
    line->setColor(1.0, 0.0, 0.0, 1.0);     // Red color for the path
    line->setScale(Ogre::Vector3(1, 1, 1)); // Set line thickness

    // Store the line in the list of lines for later clearing
    visuals_.push_back(line);
  }
}

void PathWithIdsDisplay::clearDisplay() {
  // Clear the visuals by deleting their objects.
  for (rviz::Line *line : visuals_) {
    delete line;
  }
  visuals_.clear();
}

} // end namespace rviz_path_with_ids

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_path_with_ids::PathWithIdsDisplay, rviz::Display)
