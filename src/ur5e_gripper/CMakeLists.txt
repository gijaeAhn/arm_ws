cmake_minimum_required(VERSION 2.8.3)
project(ur5e_gripper)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
    message_generation
    sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
    DIRECTORY msg
    FILES  JointState.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)



catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
    sensor_msgs
)
