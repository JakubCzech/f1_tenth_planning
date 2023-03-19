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

#ifndef F1_PLANNING__F1_PLANNING_NODE_HPP_
#define F1_PLANNING__F1_PLANNING_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "f1_planning/f1_planning.hpp"

namespace f1_planning
{
using F1PlanningPtr = std::unique_ptr<f1_planning::F1Planning>;

class F1_PLANNING_PUBLIC F1PlanningNode : public rclcpp::Node
{
public:
  explicit F1PlanningNode(const rclcpp::NodeOptions & options);

private:
  F1PlanningPtr f1_planning_{nullptr};
  void foo();
};
}  // namespace f1_planning

#endif  // F1_PLANNING__F1_PLANNING_NODE_HPP_
