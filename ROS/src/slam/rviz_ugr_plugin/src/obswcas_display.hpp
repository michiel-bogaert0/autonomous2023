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

#ifndef OBSWCAS_DISPLAY_H
#define OBSWCAS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <QObject>
#include <rviz/message_filter_display.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
} // namespace rviz

namespace rviz_observations_visualization {

class ObservationWithCovarianceVisual;

class ObservationWithCovarianceArrayStampedDisplay
    : public rviz::MessageFilterDisplay<
          ugr_msgs::ObservationWithCovarianceArrayStamped> {
  Q_OBJECT
public:
  ObservationWithCovarianceArrayStampedDisplay();
  virtual ~ObservationWithCovarianceArrayStampedDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

  // cppcheck-suppress unknownMacro
private Q_SLOTS: // cppcheck-suppress unmatchedSuppression
  void updateColorAndAlpha();
  void updateUseRealisticModel();
  void updateUseIds();

private:
  void processMessage(
      const ugr_msgs::ObservationWithCovarianceArrayStamped::ConstPtr &msg);

  std::vector<ObservationWithCovarianceVisual *> visuals_;

  rviz::ColorProperty *color_property_[5];
  rviz::FloatProperty *alpha_property_;
  rviz::BoolProperty *use_realistic_model_;
  rviz::BoolProperty *use_ids_;
  rviz::IntProperty *history_length_property_;
};

} // namespace rviz_observations_visualization

#endif