cmake_minimum_required(VERSION 3.0.2)
project(utils_ros)

find_package(catkin REQUIRED COMPONENTS sensor_msgs cv_bridge)

catkin_package(
		# INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs cv_bridge
		LIBRARIES ${PROJECT_NAME}
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/bag_to_asl.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bag_to_asl
)
