cmake_minimum_required(VERSION 3.16)
project(ik_constraint_esdf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "Python 3 interpreter")

find_package(ament_cmake REQUIRED)

set(dependencies
  coal
  Eigen3
  ik_constraint
  pinocchio
  voxblox
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/EsdfCollisionConstraint.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  coal::coal
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
