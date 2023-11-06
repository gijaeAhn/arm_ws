cmake_minimum_required(VERSION 3.20.0)
project(occ)
set(CMAKE_CXX_STANDARD 14)
set(${PROJECT_NAME}_VERSION 0.0.0)


# ADD LIBRARY
add_library(${PROJECT_NAME}_LIB SHARED
            include/Voxel_Grid.h
            include/Occupancy_Grid.h
            include/Occupancy_Map_Server.h
            src/Occupancy_Grid.cc
            src/Occupancy_Map_Server.cc
            )


