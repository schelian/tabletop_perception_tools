cmake_minimum_required(VERSION 2.8.3)
project(tabletop_perception_tools)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    rospy
    sensor_msgs
    cv_bridge
    tf
    image_transport
    pcl_ros
    )

add_message_files(
    DIRECTORY
    msg
    FILES
    Cluster.msg
    Block.msg
    Primitive.msg
)

add_service_files(
    DIRECTORY
    srv
    FILES
    ExtractClusters.srv
    FindBlocks.srv
    FindPrimitives.srv
)

generate_messages(
    DEPENDENCIES
    ${PROJECT_NAME}
    std_msgs
    sensor_msgs
    geometry_msgs
)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


include_directories(${catkin_INCLUDE_DIRS} include)
add_executable(tools_server src/tools_server.cpp)
target_link_libraries(tools_server ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
add_dependencies( tools_server ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies( tools_server ${PROJECT_NAME}_generate_messages_py )

