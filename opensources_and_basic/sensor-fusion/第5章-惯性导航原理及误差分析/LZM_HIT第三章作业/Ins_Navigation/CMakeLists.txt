cmake_minimum_required(VERSION 3.5)
project(ckitti2bag)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

set(EXTERN_LIBS "")
set(ROS_INCLUDE "")

### find ros ###
find_package(roscpp REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${roscpp_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${roscpp_LIBRARIES})

find_package(tf REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${tf_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${tf_LIBRARIES})

find_package(rosbag REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${rosbag_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${rosbag_LIBRARIES})

find_package(Boost REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${BOOST_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${BOOST_LIBRARIES} boost_filesystem)

### find opencv ###
find_package(OpenCV REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${OpenCV_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${OpenCV_Libraries})

### find PCL ###
find_package(PCL REQUIRED)
set(ROS_INCLUDE ${ROS_INCLUDE} ${PCL_INCLUDE_DIRS})
set(EXTERN_LIBS ${EXTERN_LIBS} ${PCL_Libraries})

### find YAML ###
find_package(yaml-cpp REQUIRED)
set(EXTERN_LIBS ${EXTERN_LIBS} yaml-cpp)

include_directories(include ${ROS_INCLUDE})

set(SRC_FILE 
   src/main.cpp
   src/ins_navigation_interface.cpp
   src/ins_navigation.cpp
   )

add_executable(ins_navigation src/main.cpp ${SRC_FILE})
target_link_libraries(ins_navigation ${EXTERN_LIBS})
