cmake_minimum_required(VERSION 3.16)
project(mujoco_esdf)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

set(dependencies
  Eigen3
  mujoco_vendor
  voxblox
)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} src/mujoco_esdf.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} voxblox)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  mujoco::mujoco
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
