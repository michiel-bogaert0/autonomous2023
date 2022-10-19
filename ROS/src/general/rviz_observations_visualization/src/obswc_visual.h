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

#ifndef OBSWC_VISUAL_H
#define OBSWC_VISUAL_H

#include <ugr_msgs/ObservationWithCovariance.h>

namespace Ogre
{
  class Vector3;
  class Quaternion;
}

namespace rviz
{
  class Shape;
  class Arrow;
}

namespace rviz_observations_visualization
{

  // BEGIN_TUTORIAL
  // Declare the visual class for this display.
  //
  // Each instance of ImuVisual represents the visualization of a single
  // sensor_msgs::Imu message.  Currently it just shows an arrow with
  // the direction and magnitude of the acceleration vector, but could
  // easily be expanded to include more of the message data.
  class ObservationWithCovarianceVisual
  {
  public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    ObservationWithCovarianceVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~ObservationWithCovarianceVisual();

    void setPosition(float x, float y); 
    void setCovariance(boost::array<double, 9> &covariance);

    // Set the color and alpha of the visual, which are user-editable
    // parameters and therefore don't come from the Imu message.
    void setColor(float r, float g, float b, float a);

  private:
    rviz::Arrow* cone_shape_;
    rviz::Shape* position_shape_;

    Ogre::SceneNode *frame_node_;
    Ogre::SceneNode *cone_node_;
    Ogre::SceneNode *position_node_;
    Ogre::SceneManager *scene_manager_;
  };
}

#endif