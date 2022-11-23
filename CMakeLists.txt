cmake_minimum_required(VERSION 3.0.2)
project(utils_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
            sensor_msgs cv_bridge roscpp rosbag message_filters
            image_transport imagenex831l camera_info_manager)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs cv_bridge roscpp rosbag message_filters image_transport camera_info_manager
    imagenex831l
		LIBRARIES ${PROJECT_NAME}
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/bag_to_asl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/asl_to_rosbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/bag_to_colmap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/add_camera_info.py 
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

add_library(rosdata_utils 
            src/EurocDataProvider.cpp 
            src/Definitions.cpp
            src/BagWriter.cpp
            src/Utils.cpp
            src/PoseInterpolator.cpp
            src/Trajectory.cpp
            )

target_link_libraries(rosdata_utils 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      gtsam
                      )

target_compile_options(rosdata_utils  PRIVATE -march=native)
add_executable(asl_to_rosbag src/asl_to_rosbag.cpp)
target_link_libraries(asl_to_rosbag rosdata_utils)

add_executable(add_svin_pose src/add_svin_gt.cpp)
target_link_libraries(add_svin_pose rosdata_utils)

add_executable(interpolate_trajectories src/interpolate_trajectories.cpp)
target_link_libraries(interpolate_trajectories rosdata_utils)

add_executable(write_kf_images src/write_keyframe_image.cpp)
target_link_libraries(write_kf_images rosdata_utils -lstdc++fs)

add_executable(sync_bag src/sync_bag.cpp)
target_link_libraries(sync_bag rosdata_utils -lstdc++fs)