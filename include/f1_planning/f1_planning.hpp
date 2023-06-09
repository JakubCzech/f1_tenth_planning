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

#ifndef F1_PLANNING__F1_PLANNING_HPP_
#define F1_PLANNING__F1_PLANNING_HPP_

#include <cstdint>

#include "f1_planning/visibility_control.hpp"


namespace f1_planning
{

class F1_PLANNING_PUBLIC F1Planning
{
public:
  F1Planning();
  void setParameters(int64_t param_name);

private:
  int64_t param_name_{123};
};

}  // namespace f1_planning

#endif  // F1_PLANNING__F1_PLANNING_HPP_
