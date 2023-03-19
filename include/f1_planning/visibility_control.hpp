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

#ifndef F1_PLANNING__VISIBILITY_CONTROL_HPP_
#define F1_PLANNING__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(F1_PLANNING_BUILDING_DLL) || defined(F1_PLANNING_EXPORTS)
    #define F1_PLANNING_PUBLIC __declspec(dllexport)
    #define F1_PLANNING_LOCAL
  #else  // defined(F1_PLANNING_BUILDING_DLL) || defined(F1_PLANNING_EXPORTS)
    #define F1_PLANNING_PUBLIC __declspec(dllimport)
    #define F1_PLANNING_LOCAL
  #endif  // defined(F1_PLANNING_BUILDING_DLL) || defined(F1_PLANNING_EXPORTS)
#elif defined(__linux__)
  #define F1_PLANNING_PUBLIC __attribute__((visibility("default")))
  #define F1_PLANNING_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define F1_PLANNING_PUBLIC __attribute__((visibility("default")))
  #define F1_PLANNING_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // F1_PLANNING__VISIBILITY_CONTROL_HPP_
