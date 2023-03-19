// Copyright 2023 Jakub Czech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "f1_planning/f1_planning_node.hpp"

namespace f1_planning
{

F1PlanningNode::F1PlanningNode(const rclcpp::NodeOptions & options)
:  Node("f1_planning", options)
{
  f1_planning_ = std::make_unique<f1_planning::F1Planning>();
  const int64_t param_name = this->declare_parameter("param_name", 456);
  f1_planning_->setParameters(param_name);
  this->foo();
}

void F1PlanningNode::foo()
{
  f1_planning_->printHello();
}

}  // namespace f1_planning

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(f1_planning::F1PlanningNode)
