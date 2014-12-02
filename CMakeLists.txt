cmake_minimum_required(VERSION 2.8.3)
project(tabletop_perception_tools)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    sensor_msgs
    cv_bridge
    tf
    image_transport
    pcl_ros
    )

catkin_package(
    INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(tools_server src/tools_server.cpp)
target_link_libraries(tools_server ${catkin_LIBRARIES})

