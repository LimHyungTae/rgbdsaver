cmake_minimum_required(VERSION 2.8.3)
project(rgbdsaver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_generation
    laser_geometry
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
   )

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES beginner_tutorials
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall -msse3 ${CMAKE_CXX_FLAGS}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


FILE(GLOB rgbdsaver_source
    "node/src/*.cpp"
    "node/src/*.h")

add_executable(rgbdsaver node/main.cpp ${rgbdsaver_source} )
add_dependencies(rgbdsaver rgbdsaver_generate_messages_cpp)
target_link_libraries(rgbdsaver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
