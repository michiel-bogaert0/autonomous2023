/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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

#include "managed_nodehandle.hpp"
#include "ros/callback_queue.h"
#include "ros/service.h"
#include "ros/this_node.h"
#include "ros/timer_manager.h"

#include "ros/rate.h"
#include "ros/time.h"

#include "XmlRpc.h"
#include "ros/init.h"
#include "ros/master.h"
#include "ros/names.h"
#include "ros/param.h"
#include "ros/service_manager.h"
#include "ros/this_node.h"
#include "ros/topic_manager.h"
#include "ros/xmlrpc_manager.h"

#include <std_msgs/String.h>

#include <boost/thread.hpp>

namespace ugr {

ManagedNodeHandle::ManagedNodeHandle(ros::NodeHandle &n) : n(n) {

  // Create publisher and service
  this->state_publisher = n.advertise<std_msgs::String>("/state", 1, true);
  this->state = Unconfigured;
  FD
}

// Publisher ManagedNodeHandle::advertise(AdvertiseOptions &ops)
// {
//   ops.topic = resolveName(ops.topic);
//   if (ops.callback_queue == 0)
//   {
//     if (callback_queue_)
//     {
//       ops.callback_queue = callback_queue_;
//     }
//     else
//     {
//       ops.callback_queue = getGlobalCallbackQueue();
//     }
//   }

//   SubscriberCallbacksPtr callbacks(
//       new SubscriberCallbacks(ops.connect_cb, ops.disconnect_cb,
//                               ops.tracked_object, ops.callback_queue));

//   if (TopicManager::instance()->advertise(ops, callbacks))
//   {
//     Publisher pub(ops.topic, ops.md5sum, ops.datatype, *this, callbacks);

//     {
//       boost::mutex::scoped_lock lock(collection_->mutex_);
//       collection_->pubs_.push_back(pub.impl_);
//     }

//     return pub;
//   }

//   return Publisher();
// }

// Subscriber ManagedNodeHandle::subscribe(SubscribeOptions &ops)
// {
//   ops.topic = resolveName(ops.topic);
//   if (ops.callback_queue == 0)
//   {
//     if (callback_queue_)
//     {
//       ops.callback_queue = callback_queue_;
//     }
//     else
//     {
//       ops.callback_queue = getGlobalCallbackQueue();
//     }
//   }

//   if (TopicManager::instance()->subscribe(ops))
//   {
//     Subscriber sub(ops.topic, *this, ops.helper);

//     {
//       boost::mutex::scoped_lock lock(collection_->mutex_);
//       collection_->subs_.push_back(sub.impl_);
//     }

//     return sub;
//   }

//   return Subscriber();
// }

} // namespace ugr