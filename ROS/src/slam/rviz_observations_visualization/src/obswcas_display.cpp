/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include "obswc_visual.h"
#include "obswcas_display.h"

namespace rviz_observations_visualization {

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ObservationWithCovarianceArrayStampedDisplay::
    ObservationWithCovarianceArrayStampedDisplay() {
  color_property_[0] =
      new rviz::ColorProperty("Class 0 color", QColor(0, 0, 255),
                              "The color of observation class 0: 'blue'", this,
                              SLOT(updateColorAndAlpha()));

  color_property_[1] =
      new rviz::ColorProperty("Class 1 color", QColor(255, 214, 0),
                              "The color of observation class 1: 'yellow'",
                              this, SLOT(updateColorAndAlpha()));

  color_property_[2] =
      new rviz::ColorProperty("Class 2 color", QColor(255, 140, 0),
                              "The color of observation class 2: 'big orange'",
                              this, SLOT(updateColorAndAlpha()));

  color_property_[3] = new rviz::ColorProperty(
      "Class 3 color", QColor(255, 69, 0),
      "The color of observation class 3: 'small orange'", this,
      SLOT(updateColorAndAlpha()));

  color_property_[4] =
      new rviz::ColorProperty("Class 4 color", QColor(0, 0, 0),
                              "The color of observation class 4: 'unkown'",
                              this, SLOT(updateColorAndAlpha()));

  use_realistic_model_ = new rviz::BoolProperty(
      "Realistic", false,
      "Use a realistic cone model. If true, alpha and color properties do "
      "nothing. Falls back to simple models if no .dae file is available",
      this, SLOT(updateUseRealisticModel()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
}

void ObservationWithCovarianceArrayStampedDisplay::onInitialize() {
  MFDClass::onInitialize();
}

// This function MUST exist. Otherwise Rviz's plugin factory will complain
ObservationWithCovarianceArrayStampedDisplay::
    ~ObservationWithCovarianceArrayStampedDisplay() {}

// This one must also exist. Otherwise the option will not appear in GUI
void ObservationWithCovarianceArrayStampedDisplay::updateUseRealisticModel() {}

// Clear the visuals by deleting their objects.
void ObservationWithCovarianceArrayStampedDisplay::reset() {
  MFDClass::reset();
  for (auto visual : visuals_) {
    delete visual;
  }
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void ObservationWithCovarianceArrayStampedDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();

  for (size_t i = 0; i < visuals_.size(); i++) {
    Ogre::ColourValue color =
        color_property_[visuals_[i]->getVisualClass()]->getOgreColor();
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

// cppcheck-suppress unusedFunction
void ObservationWithCovarianceArrayStampedDisplay::processMessage(
    const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &msg) {

  // Get transformation
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  for (auto visual : visuals_) {
    delete visual;
  }
  visuals_.clear();

  for (ugr_msgs::ObservationWithCovariance observation : msg->observations) {

    ObservationWithCovarianceVisual *visual =
        new ObservationWithCovarianceVisual(
            scene_manager_, scene_node_,
            observation.observation.observation_class,
            use_realistic_model_->getBool());

    visual->setVisualClass(observation.observation.observation_class);
    visual->setPosition(position.x, position.y);
    visual->setLocalPosition(observation.observation.location.x,
                             observation.observation.location.y);
    visual->setOrientation(orientation);
    visual->setCovariance(observation.covariance);

    Ogre::ColourValue color =
        color_property_[observation.observation.observation_class]
            ->getOgreColor();
    visual->setColor(color.r, color.g, color.b, observation.observation.belief);

    visuals_.push_back(visual);
  }
}
} // namespace rviz_observations_visualization
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_observations_visualization::
                           ObservationWithCovarianceArrayStampedDisplay,
                       rviz::Display)