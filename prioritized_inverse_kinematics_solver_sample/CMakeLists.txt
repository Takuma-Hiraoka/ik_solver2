cmake_minimum_required(VERSION 3.16)
project(prioritized_inverse_kinematics_solver_sample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

set(dependencies
  ik_constraint_coal
  ik_constraint_vclip
  mujoco_viewer
  pinocchio
  ik_constraint
  prioritized_inverse_kinematics_solver
  ament_index_cpp
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

set(common_dependencies
  mujoco_viewer
  ik_constraint
  prioritized_inverse_kinematics_solver
  ament_index_cpp
)

add_executable(com src/com.cpp)
add_executable(collision src/collision.cpp)
add_executable(multi_robot src/multi_robot.cpp)
add_executable(position src/position.cpp)

foreach(target com multi_robot position)
  ament_target_dependencies(${target}
    ${common_dependencies}
    )

  target_link_libraries(${target}
    pinocchio::pinocchio
    )
endforeach()

ament_target_dependencies(collision
  ${dependencies}
  )

target_link_libraries(collision
  pinocchio::pinocchio
  )

install(TARGETS
  collision
  com
  multi_robot
  position
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY unitree_ros
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
