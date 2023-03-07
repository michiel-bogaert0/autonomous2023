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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/mesh_loader.h>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "obswc_visual.h"

namespace rviz_observations_visualization
{

  // Credits: https://github.com/laas/rviz_plugin_covariance
  // Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
  void makeRightHanded(Eigen::Matrix2d &eigenvectors, Eigen::Vector2d &eigenvalues)
  {
    // Note that sorting of eigenvalues may end up with left-hand coordinate system.
    // So here we correctly sort it so that it does end up being righ-handed and normalised.
    Eigen::Vector3d c0;
    c0.setZero();
    c0.head<2>() = eigenvectors.col(0);
    c0.normalize();
    Eigen::Vector3d c1;
    c1.setZero();
    c1.head<2>() = eigenvectors.col(1);
    c1.normalize();
    Eigen::Vector3d cc = c0.cross(c1);
    if (cc[2] < 0)
    {
      eigenvectors << c1.head<2>(), c0.head<2>();
      double e = eigenvalues[0];
      eigenvalues[0] = eigenvalues[1];
      eigenvalues[1] = e;
    }
    else
    {
      eigenvectors << c0.head<2>(), c1.head<2>();
    }
  }

  void computeShapeScaleAndOrientation2D(const Eigen::Matrix2d &covariance, Ogre::Vector3 &scale, Ogre::Quaternion &orientation)
  {
    Eigen::Vector2d eigenvalues(Eigen::Vector2d::Identity());
    Eigen::Matrix2d eigenvectors(Eigen::Matrix2d::Zero());

    // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
    // FIXME: Should we use Eigen's pseudoEigenvectors() ?
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
    // Compute eigenvectors and eigenvalues
    if (eigensolver.info() == Eigen::Success)
    {
      eigenvalues = eigensolver.eigenvalues();
      eigenvectors = eigensolver.eigenvectors();
    }
    else
    {
      ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
      eigenvalues = Eigen::Vector2d::Zero(); // Setting the scale to zero will hide it on the screen
      eigenvectors = Eigen::Matrix2d::Identity();
    }

    // Be sure we have a right-handed orientation system
    makeRightHanded(eigenvectors, eigenvalues);

    orientation.FromRotationMatrix(Ogre::Matrix3(eigenvectors(0, 0), eigenvectors(0, 1), 0,
                                                 eigenvectors(1, 0), eigenvectors(1, 1), 0,
                                                 0, 0, 1));

    scale.x = 2 * std::sqrt(eigenvalues[0]);
    scale.y = 2 * std::sqrt(eigenvalues[1]);
    scale.z = 0;
  }

  ObservationWithCovarianceVisual::ObservationWithCovarianceVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node, unsigned int cls, bool realistic)
  {
    visual_class_ = cls;
    realistic_ = realistic;

    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    cone_node_ = frame_node_->createChildSceneNode();

    // Only supports classes 0 through 3 (inclusive)
    if (!realistic_ || cls > 3)
    {
      realistic_ = false;
      cone_shape_ = new rviz::Arrow(scene_manager_, cone_node_, 0.0, 0.0, 0.33, 0.23);

      Ogre::Vector3 direction;
      direction.x = 0.0;
      direction.y = 0.0;
      direction.z = 1.0;
      cone_shape_->setDirection(direction);
    }
    else
    {

      std::string resource = "package://rviz_observations_visualization/media/class_" + std::to_string(cls) + ".dae";

      if (rviz::loadMeshFromResource("package://rviz_observations_visualization/media/class_" + std::to_string(cls) + ".dae").isNull())
      {
        ROS_ERROR("Failed to load model resource '%s'.", resource.c_str());
        return;
      }

      Ogre::Entity *entity = scene_manager_->createEntity(resource);
      cone_node_->attachObject(entity);
    }

    position_node_ = frame_node_->createChildSceneNode();
    position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);
  }

  ObservationWithCovarianceVisual::~ObservationWithCovarianceVisual()
  {
    if (!realistic_)
    {
      delete cone_shape_;
    }

    scene_manager_->destroySceneNode(cone_node_);

    delete position_shape_;

    scene_manager_->destroySceneNode(position_node_);
    scene_manager_->destroySceneNode(frame_node_);
  }

  void ObservationWithCovarianceVisual::setPosition(float x, float y)
  {
    frame_node_->setPosition(x, y, 0.0);
  }
  void ObservationWithCovarianceVisual::setLocalPosition(float x, float y)
  {
    position_node_->setPosition(x, y, 0.0);
    cone_node_->setPosition(x, y, 0.0);
  }

  void ObservationWithCovarianceVisual::setOrientation(Ogre::Quaternion orientation)
  {
    frame_node_->setOrientation(orientation);
  }

  void ObservationWithCovarianceVisual::setCovariance(boost::array<double, 9> &covariance)
  {
    Ogre::Vector3 shape_scale;
    Ogre::Quaternion shape_orientation;
    Eigen::Matrix2d covarianceMatrix(2, 2);
    covarianceMatrix(0, 0) = covariance[0];
    covarianceMatrix(0, 1) = covariance[1];
    covarianceMatrix(1, 0) = covariance[3];
    covarianceMatrix(1, 1) = covariance[4];

    computeShapeScaleAndOrientation2D(covarianceMatrix, shape_scale, shape_orientation);
    shape_scale.z = 0.001;

    position_node_->setOrientation(shape_orientation);
    if (!shape_scale.isNaN())
      position_node_->setScale(shape_scale);
    else
      ROS_WARN_STREAM("position shape_scale contains NaN: " << shape_scale);
  }

  void ObservationWithCovarianceVisual::setColor(float r, float g, float b, float a)
  {
    if (!realistic_)
    {
      cone_shape_->setColor(r, g, b, a);
    }
    position_shape_->setColor(r, g, b, a);
  }
}