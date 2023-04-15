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

#include "f1_planning/f1_planning.hpp"

#include <iostream>

namespace f1_planning
{

F1Planning::F1Planning()
{
}

void F1Planning::setParameters(int64_t param_name)
{
  param_name_ = param_name;
}


}  // namespace f1_planning
