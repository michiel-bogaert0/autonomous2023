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

#include <iostream>
#include <memory>
#include <thread>

#include "ros/ros.h"
#include "tf_service/buffer_server.h"

#include "boost/program_options.hpp"

namespace po = boost::program_options;

int main(int argc, char** argv) {
  int num_threads = 0;

  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
    ("help", "show usage")
    ("num_threads", po::value<int>(&num_threads)->default_value(0),
     "Number of handler threads. 0 means number of CPU cores.")
    ("cache_time", po::value<double>(),
     "Buffer cache time of the underlying TF buffer in seconds.")
    ("max_timeout", po::value<double>(),
     "Requests with lookup timeouts (seconds) above this will be blocked.")
    ("frames_service", "Advertise the tf2_frames service.")
    ("debug", "Advertise the tf2_frames service (same as --frames_service).")
    ("add_legacy_server", "If set, also run a tf2_ros::BufferServer.")
    ("legacy_server_namespace", po::value<std::string>(),
     "Use a separate namespace for the legacy action server.")
  ;
  // clang-format on
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  } catch (const po::error& exception) {
    std::cerr << exception.what() << std::endl;
    return EXIT_FAILURE;
  }
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "tf_service");

  // boost::po overflows unsigned int for negative values passed to argv,
  // so we use a signed one and check manually.
  if (num_threads < 0) {
    ROS_ERROR("The number of threads can't be negative.");
    return EXIT_FAILURE;
  } else if (num_threads == 0) {
    ROS_INFO_STREAM("--num_threads unspecified / zero, using available cores.");
    num_threads = std::thread::hardware_concurrency();
  }

  tf_service::ServerOptions options;
  if (vm.count("cache_time"))
    options.cache_time = ros::Duration(vm["cache_time"].as<double>());
  if (vm.count("max_timeout"))
    options.max_timeout = ros::Duration(vm["max_timeout"].as<double>());
  options.debug = vm.count("frames_service") || vm.count("debug");
  options.add_legacy_server = vm.count("add_legacy_server");
  options.legacy_server_namespace =
      vm.count("legacy_server_namespace")
          ? vm["legacy_server_namespace"].as<std::string>()
          : ros::this_node::getName();

  ROS_INFO_STREAM("Starting server with " << num_threads << " handler threads");
  ROS_INFO_STREAM_COND(options.add_legacy_server,
                       "Also starting a legacy tf2::BufferServer in namespace "
                           << options.legacy_server_namespace);
  tf_service::Server server(options);
  ros::AsyncSpinner spinner(num_threads);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();

  return EXIT_SUCCESS;
}
