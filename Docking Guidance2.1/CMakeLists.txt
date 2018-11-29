cmake_minimum_required(VERSION 3.9)
project(Docking_Guidance)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-fext-numeric-literals)

add_subdirectory(pandar_grabber)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Docking_Guidance Docking_Guidance.cpp ProcessPointcloud.h ProcessPointCloud.cpp Cluster.cpp Cluster.h framework.h framework.cpp lidar_detect.h  WalkTest.cpp WalkTest.h tools_function.h tools_function.cpp GlobleData.cpp GlobleData.cpp GlobleData.h)
target_link_libraries(Docking_Guidance pandar_grabber)
target_link_libraries(Docking_Guidance ${PCL_LIBRARIES})
target_link_libraries(Docking_Guidance -lpthread sqlite3)


