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

#ifndef ROSCPP_MANAGED_NODE_HANDLE_H
#define ROSCPP_MANAGED_NODE_HANDLE_H

#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/common.h"
#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/service_client_options.h"
#include "ros/service_server.h"
#include "ros/spinner.h"
#include "ros/subscribe_options.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "ros/timer_options.h"
#include "ros/wall_timer.h"
#include "ros/wall_timer_options.h"
#include <ugr_msgs/State.h>

#include <boost/bind.hpp>

#include <XmlRpcValue.h>

namespace ugr {

/**
 * \brief Enumerator for the managed node state machine
 */
enum ManagedNodeState { Unconfigured, Inactive, Active, Finalized };

/**
 * \brief Strings corresponding to the state machine enumerator
 */
const char *ManagedNodeStateStrings[] = {"Unconfigured", "Inactive", "Active",
                                         "Finalized"};

class ManagedNodeHandle {
public:
  explicit ManagedNodeHandle(ros::NodeHandle &n);

  /**
   * \brief Advertise a topic, with full range of AdvertiseOptions
   *
   * This call connects to the master to publicize that the node will be
   * publishing messages on the given topic.  This method returns a Publisher
that allows you to
   * publish a message on this topic.
   *
   * This is an advanced version advertise() that exposes all options (through
the AdvertiseOptions structure)
   *
   * \param ops Advertise options to use
   * \return On success, a Publisher that, when it goes out of scope, will
automatically release a reference
   * on this advertisement.  On failure, an empty Publisher which can be checked
with: \verbatim if (handle)
{
...
}
\endverbatim
   *
   * \throws InvalidNameException If the topic name begins with a tilde, or is
an otherwise invalid graph resource name
   */
  // Publisher advertise(AdvertiseOptions &ops);

  /**
   * \brief Subscribe to a topic, version with full range of SubscribeOptions
   *
   * This method connects to the master to register interest in a given
   * topic.  The node will automatically be connected with publishers on
   * this topic.  On each message receipt, fp is invoked and passed a shared
pointer
   * to the message received.  This message should \b not be changed in place,
as it
   * is shared with any other subscriptions to this topic.
   *
   * This version of subscribe allows the full range of options, exposed through
the SubscribeOptions class
   *
   * \param ops Subscribe options
   * \return On success, a Subscriber that, when all copies of it go out of
scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is
an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already
subscribed to the same topic with a different datatype
   */
  // Subscriber subscribe(SubscribeOptions &ops);

private:
  ros::NodeHandle &n;

  ros::Publisher state_publisher;

  ManagedNodeState state;
};

} // namespace ugr

#endif // ROSCPP_MANAGED_NODE_HANDLE_H