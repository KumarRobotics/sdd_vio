cmake_minimum_required(VERSION 2.8.3)
project(sdd_vio)

IF(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -march=native -mfpu=neon")
  #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Ofast -fno-signed-zeros -fno-math-errno -funroll-loops -fno-strict-aliasing")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11")
ENDIF()

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  visualization_msgs
  tf
  nodelet
  rosbag
)

FIND_PACKAGE(OpenCV 3 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
#find_package(Ceres REQUIRED)


# Check if OpenCV package has been found
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Check if Eigen package has been found
message(STATUS "Eigen library status:")
message(STATUS "    version: ${EIGEN3_VERSION}")
message(STATUS "    libraries: ${EIGEN3_LIBS}")
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIRS}")

catkin_package(
  LIBRARIES vo_nodelet imu_integration_nodelet
  DEPENDS EIGEN3 OpenCV
  CATKIN_DEPENDS roscpp image_transport sensor_msgs visualization_msgs tf nodelet
  LIBRARIES sdd_vio_nodelet
)

INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pinhole_camera src/pinhole_camera_stereo.cpp)
target_link_libraries(pinhole_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} utils)

add_library(utils src/utils/math_utils.cpp src/utils/calib_utils.cpp)
target_link_libraries(utils ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_library(vo src/vo_stereo_1.cpp src/vo_stereo_2.cpp src/vo_stereo_3.cpp src/vo_stereo_4.cpp src/grid.cpp)
# add_library(vo src/disp_vo_stereo.cpp src/grid.cpp)
target_link_libraries(vo utils pinhole_camera ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})  # ${CERES_LIBRARIES}


# for nodelet
add_library(vo_nodelet src/sdd_vio_nodelet.cpp src/visualization.cpp)
add_dependencies(vo_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vo_nodelet vo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(imu_integration_nodelet src/imu_integration_nodelet.cpp)
add_dependencies(imu_integration_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_integration_nodelet ${catkin_LIBRARIES})


# for node
ADD_EXECUTABLE(sdd_vio_node src/sdd_vio_node.cpp src/visualization.cpp)

TARGET_LINK_LIBRARIES(
  sdd_vio_node
  pinhole_camera
  vo
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)


add_executable(sdd_vio_bag_reader src/sdd_vio_bag_reader.cpp)
target_link_libraries(sdd_vio_bag_reader vo_nodelet)
