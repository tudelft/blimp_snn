cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf roslaunch)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(pose_stamper_1 src/pose_stamper_1.cpp)
target_link_libraries(pose_stamper_1 ${catkin_LIBRARIES})
#add_executable(mocap_node src/mocap_node.cpp)
#target_link_libraries(mocap_node ${catkin_LIBRARIES})

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)
