cmake_minimum_required(VERSION 3.0.2)
project(controller_pkg)
add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs trajectory_msgs mav_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS
  roscpp geometry_msgs nav_msgs trajectory_msgs mav_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_eigen DEPENDS EIGEN3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

foreach(node controller_node traj_publisher)
  add_executable(${node} src/${node}.cpp)
  add_dependencies(${node} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node} ${catkin_LIBRARIES})
endforeach()

install(TARGETS controller_node traj_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/rigid_body_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
