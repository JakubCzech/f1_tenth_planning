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

#include "gtest/gtest.h"
#include "f1_planning/f1_planning.hpp"

TEST(TestF1Planning, TestHello) {
  std::unique_ptr<f1_planning::F1Planning> f1_planning_ =
    std::make_unique<f1_planning::F1Planning>();
  auto result = f1_planning_->printHello();
  EXPECT_EQ(result, 123);
}