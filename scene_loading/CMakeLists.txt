cmake_minimum_required(VERSION 2.8.3)
project(scene_loading)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)
