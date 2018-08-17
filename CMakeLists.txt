project(demo_rgbd_new)

cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -g -O3")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set(OpenCV_DIR "/home/davidz/work/3rdlibs/OpenCV-2.4.10/build")
find_package(OpenCV REQUIRED)
# message("OpenCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIRS}")
# include_directories("/home/hzhang8/work/3rdlib/opencv-3.3.1/install/include")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
    # camera_model
    tf
)

catkin_package(
    # DEPENDS Eigen
    # CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
    # INCLUDE_DIRS include
    # LIBRARIES
)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 QUIET)
include_directories(${EIGEN3_INCLUDE_DIR} "/usr/include/eigen3")
# include_directories("/usr/include/eigen3")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories(${catkin_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# add_subdirectory(isamlib)
# link_libraries(isamlib)
 link_libraries(${CERES_LIBRARIES})
 link_libraries(${catkin_LIBRARIES})
 link_libraries(${OpenCV_LIBS})

## feature tracking 
add_executable(featureTracking_node feature_tracking/feature_tracker.cpp feature_tracking/feature_tracker_node.cpp)
add_executable(test_feat_tracker feature_tracking/feature_tracker.cpp feature_tracking/test_feat_track.cpp)
add_executable(featureTracking_ori feature_tracking/featureTracking.cpp)

## vo 
add_executable(test_vo_euler vo/test_vo_euler.cpp vo/projection_euler-ZXY.cpp)
add_executable(test_vo_euler_ip vo/test_vo_euler_ip.cpp vo/projection_euler-ZXY.cpp)
add_executable(test_vo_quaternion vo/test_vo_quaternion.cpp vo/projection_quat.cpp)
add_executable(test_vo_quaternion_ip vo/test_vo_quaternion_ip.cpp vo/projection_quat.cpp)
add_executable(visual_odometry vo/visual_odometry.cpp vo/visual_odometry_node.cpp vo/stereo_triangulate.cpp)

## depth handler
add_executable(depth_handler vo/depth_handler_node.cpp vo/depth_handler.cpp )
add_executable(test_depth_process vo/test_depth_process.cpp)

# target_link_libraries(featureTracking ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_executable(visualOdometry src/visualOdometry.cpp)
# target_link_libraries(visualOdometry ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_executable(bundleAdjust src/bundleAdjust.cpp)
# target_link_libraries(bundleAdjust isamlib ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_executable(processDepthmap src/processDepthmap.cpp)
# add_executable(stackDepthPoint src/stackDepthPoint.cpp)
# add_executable(transformMaintenance src/transformMaintenance.cpp)
# add_executable(registerPointCloud src/registerPointCloud.cpp)

