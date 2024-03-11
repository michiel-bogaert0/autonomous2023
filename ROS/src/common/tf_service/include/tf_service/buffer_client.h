// Copyright 2019 Magazino GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/buffer_interface.h"

namespace tf_service {

class BufferClient : public tf2_ros::BufferInterface {
 public:
  BufferClient() = delete;

  explicit BufferClient(const std::string& server_node_name);

  BufferClient(const std::string& server_node_name,
               const ros::Duration keepalive_period);

  ~BufferClient();

  geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const ros::Time& time, const ros::Duration timeout) const override;

  geometry_msgs::TransformStamped lookupTransform(
      const std::string& target_frame, const ros::Time& target_time,
      const std::string& source_frame, const ros::Time& source_time,
      const std::string& fixed_frame,
      const ros::Duration timeout) const override;

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const ros::Time& time,
                    const ros::Duration timeout,
                    std::string* errstr = NULL) const override;

  bool canTransform(const std::string& target_frame,
                    const ros::Time& target_time,
                    const std::string& source_frame,
                    const ros::Time& source_time,
                    const std::string& fixed_frame, const ros::Duration timeout,
                    std::string* errstr = NULL) const override;

  bool reconnect(ros::Duration timeout = ros::Duration(10));
  void asyncReconnect(const ros::Duration timeout = ros::Duration(-1));
  bool isConnected() const;
  bool waitForServer(const ros::Duration timeout = ros::Duration(-1));

 private:
  mutable std::mutex mutex_;
  mutable std::mutex reconnection_mutex_;

  ros::NodeHandle node_handle_;

  // mutable because ServiceClient::call() isn't const.
  mutable ros::ServiceClient can_transform_client_;     // GUARDED_BY(mutex_);
  mutable ros::ServiceClient lookup_transform_client_;  // GUARDED_BY(mutex_);

  std::future<bool> async_reconnected_;
  ros::Timer keepalive_timer_;
};

}  // namespace tf_service
