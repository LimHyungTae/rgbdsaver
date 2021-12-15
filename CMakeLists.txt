cmake_minimum_required(VERSION 2.8.3)
project(rgbdsaver)

add_compile_options(-std=c++17)

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

add_message_files(
   FILES
   node.msg
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
find_package(roscpp)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(sensor_msgs)
find_package(PCL 1.8 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall -msse3 ${CMAKE_CXX_FLAGS}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

FILE(GLOB datasaver_source
    "nodes/datasaver/*.cpp"
    "nodes/datasaver/*.h")

add_executable(rgbdsaver nodes/save.cpp ${datasaver_source} )
add_dependencies(rgbdsaver rgbdsaver_generate_messages_cpp)
target_link_libraries(rgbdsaver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

FILE(GLOB nodegen_source
        "nodes/nodegen/*.cpp"
        "nodes/nodegen/*.h")

add_executable(nodegen nodes/sync.cpp ${nodegen_source} )
add_dependencies(nodegen rgbdsaver_generate_messages_cpp)
target_link_libraries(nodegen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

