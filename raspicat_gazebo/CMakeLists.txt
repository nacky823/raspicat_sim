cmake_minimum_required(VERSION 3.8)
project(raspicat_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake_auto REQUIRED)
find_package(gazebo REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(raspicat_sim
  src/raspicat_sim.cpp
)

add_library(follow_actor SHARED src/follow_actor_plugin.cpp)
target_link_libraries(follow_actor ${GAZEBO_LIBRARIES})
target_include_directories(follow_actor PRIVATE ${GAZEBO_INCLUDE_DIRS})
set_target_properties(follow_actor PROPERTIES OUTPUT_NAME "follow_actor")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS follow_actor
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/download_gazebo_models.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  worlds
)