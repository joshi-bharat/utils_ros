cmake_minimum_required(VERSION 3.0.2)
project(utils_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
            sensor_msgs cv_bridge roscpp rosbag)
find_package(GTSAM REQUIRED)
find_package(OpenCV 3.2.0 EXACT REQUIRED)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs cv_bridge roscpp rosbag
		LIBRARIES ${PROJECT_NAME}
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/bag_to_asl.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bag_to_asl
)

catkin_install_python(PROGRAMS
  scripts/asl_to_rosbag.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/asl_to_rosbag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(euroc_data_provider 
            src/EurocDataProvider.cpp 
            src/Definitions.cpp
            src/BagWriter.cpp)

target_link_libraries(euroc_data_provider 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      gtsam)

# target_compile_options(euroc_data_provider  PRIVATE -march=native)
add_executable(asl_to_rosbag src/ASLtoRosBag.cpp)
target_link_libraries(asl_to_rosbag euroc_data_provider)

