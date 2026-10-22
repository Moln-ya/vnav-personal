cmake_minimum_required(VERSION 3.0.2)
project(mav_trajectory_generation)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS mav_msgs)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_library(GLOG_LIBRARY glog REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARY} ${NLOPT_LIBRARY}
  CATKIN_DEPENDS mav_msgs DEPENDS EIGEN3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/motion_defines.cpp src/polynomial.cpp src/segment.cpp
  src/timing.cpp src/trajectory.cpp src/trajectory_sampling.cpp src/vertex.cpp
  src/io.cpp src/rpoly/rpoly_ak1.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARY} ${NLOPT_LIBRARY} ${YAML_CPP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
