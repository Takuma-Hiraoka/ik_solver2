cmake_minimum_required(VERSION 3.16)
project(prioritized_inverse_kinematics_solver_sample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

set(dependencies
  mujoco_viewer
  pinocchio
  ik_constraint
  prioritized_inverse_kinematics_solver
  ament_index_cpp
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_executable(position src/position.cpp)

ament_target_dependencies(position
  ${dependencies}
  )

target_link_libraries(position
  pinocchio::pinocchio
  )

install(TARGETS
  position
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY unitree_ros
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
