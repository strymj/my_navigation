cmake_minimum_required(VERSION 2.8.3)
project(my_navigation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	actionlib
	tf
	mylib
	)

catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES nav_param
	#  CATKIN_DEPENDS roscpp rospy std_msgs
	#  DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

add_executable(goal_publisher
	src/goal_publisher.cpp)
target_link_libraries(goal_publisher
	${catkin_LIBRARIES}
	mylib
	)

# add_executable(map_changer
# 	src/map_changer.cpp)
# target_link_libraries(map_changer
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBRARIES}
# 	lib2dl
# 	)
#
# add_executable(polegoal_publisher
# 	src/polegoal_publisher.cpp)
# target_link_libraries(polegoal_publisher
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBRARIES}
# 	lib2dl
# 	)

add_executable(pet_detector
	src/pet_detector.cpp)
target_link_libraries(pet_detector
	${catkin_LIBRARIES}
	mylib
	)

add_executable(ojm_detector
	src/ojm_detector.cpp)
target_link_libraries(ojm_detector
	${catkin_LIBRARIES}
	mylib
	)

add_executable(mark_detector
	src/mark_detector.cpp)
target_link_libraries(mark_detector
	${catkin_LIBRARIES}
	)

add_executable(mark_sample
	src/mark_sample.cpp)
target_link_libraries(mark_sample
	${catkin_LIBRARIES}
	)
