// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/plugins/decorator/run_once_per_epoch.hpp"

namespace pb2025_sentry_behavior
{
RunOncePerEpoch::RunOncePerEpoch(const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList RunOncePerEpoch::providedPorts()
{
  return {BT::InputPort<uint64_t>("epoch", "{@motion_epoch}")};
}

BT::NodeStatus RunOncePerEpoch::tick()
{
  const auto epoch = getInput<uint64_t>("epoch");
  if (!epoch) {
    return BT::NodeStatus::FAILURE;
  }

  if (!has_epoch_ || epoch.value() != last_epoch_) {
    resetChild();
    has_epoch_ = true;
    last_epoch_ = epoch.value();
    completed_ = false;
  }

  if (completed_) {
    return BT::NodeStatus::SKIPPED;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const auto child_status = child_node_->executeTick();
  if (BT::isStatusCompleted(child_status)) {
    completed_ = true;
    resetChild();
  }
  return child_status;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::RunOncePerEpoch>("RunOncePerEpoch");
}
