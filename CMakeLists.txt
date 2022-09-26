cmake_minimum_required (VERSION 2.8)

project(LaserScanAccumulator)

include_directories(
    ./include
    /opt/ros/noetic/include
)

file(GLOB catkin_LIBRARIES
     "/opt/ros/noetic/lib/*.so"
)

add_executable(accum src/main.cpp)
target_link_libraries(accum ${catkin_LIBRARIES})
