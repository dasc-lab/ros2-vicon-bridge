cmake_minimum_required(VERSION 3.16)


project(vicon_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################
# DEPENDENCIES #
################

find_package(Boost REQUIRED COMPONENTS thread date_time chrono)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 3.4 NO_MODULE)
find_package(sensor_msgs REQUIRED)

include_directories(
	${Boost_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS}
	vicon_sdk/DataStream
	vicon_sdk/DataStream/ViconDataStreamSDK_CPP
	vicon_sdk
	include
	)

#############
# VICON SDK #
#############

# Compile ViconSDK from source
file(GLOB_RECURSE vicon_sdk_files "${CMAKE_CURRENT_SOURCE_DIR}/vicon_sdk/**/**.cpp")
add_library(vicon_sdk ${vicon_sdk_files})
target_link_libraries(vicon_sdk PUBLIC ${Boost_LIBRARIES})


###########################
# NON-ROS TEST EXECUTABLE #
###########################
add_executable(vicon_test src/vicon_test.cpp)
target_link_libraries(vicon_test vicon_sdk)

add_executable(vicon_test_fps src/vicon_test_fps.cpp)
target_link_libraries(vicon_test_fps vicon_sdk)

add_executable(test_perturb src/test_perturb.cpp)
target_link_libraries(test_perturb Eigen3::Eigen)

##################
# ROS EXECUTABLE #
##################

add_executable(vicon_bridge src/vicon_bridge.cpp)
ament_target_dependencies(vicon_bridge
	rclcpp
	tf2
	tf2_ros
	geometry_msgs
	sensor_msgs
	diagnostic_updater
)
target_link_libraries(vicon_bridge vicon_sdk ${Boost_LIBRARIES})

add_executable(vicon_bridge_retiming src/vicon_bridge_retiming.cpp)
ament_target_dependencies(vicon_bridge_retiming
	rclcpp
	tf2
	tf2_ros
	geometry_msgs
)
target_link_libraries(vicon_bridge_retiming 
	vicon_sdk 
	${Boost_LIBRARIES}
)

add_executable(noisy_vicon src/noisy_vicon.cpp)
ament_target_dependencies(noisy_vicon
	rclcpp
	geometry_msgs
	tf2_ros
	)
target_link_libraries(noisy_vicon
	Eigen3::Eigen
)

install(TARGETS
	vicon_bridge
       	vicon_bridge_retiming
	noisy_vicon
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)




ament_package()
