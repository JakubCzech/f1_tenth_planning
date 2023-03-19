cmake_minimum_required(VERSION 3.14)
project(f1_planning)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(F1_PLANNING_LIB_SRC
  src/f1_planning.cpp
)

set(F1_PLANNING_LIB_HEADERS
  include/f1_planning/f1_planning.hpp
  include/f1_planning/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${F1_PLANNING_LIB_SRC}
  ${F1_PLANNING_LIB_HEADERS}
)

set(F1_PLANNING_NODE_SRC
  src/f1_planning_node.cpp
)

set(F1_PLANNING_NODE_HEADERS
  include/f1_planning/f1_planning_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${F1_PLANNING_NODE_SRC}
  ${F1_PLANNING_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "f1_planning::F1PlanningNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_f1_planning.cpp)
  set(TEST_F1_PLANNING_EXE test_f1_planning)
  ament_add_gtest(${TEST_F1_PLANNING_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_F1_PLANNING_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
