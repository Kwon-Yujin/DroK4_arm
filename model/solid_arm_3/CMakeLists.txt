cmake_minimum_required(VERSION 2.8.3)

project(solid_arm_3)

find_package(catkin REQUIRED)

catkin_package()


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  roslaunch
)

find_package(gazebo REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES   solid_arm_3
    DEPENDS gazebo_ros roscpp rospy std_msgs
    DEPENDS system_lib
)


include_directories(${Boost_include_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/drok4_plugin.cc)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
