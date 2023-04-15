cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable (iterative_closest_point iterative_closest_point.cpp)
# target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

# add_executable (filter filter.cpp)
# target_link_libraries (filter ${PCL_LIBRARIES})

add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})
target_link_libraries(test -lX11)

add_executable (test_1 test_1.cpp)
target_link_libraries (test_1 ${PCL_LIBRARIES})

# add_executable (move_ground RANSAC2filter_ground.cpp)
# target_link_libraries (move_ground ${PCL_LIBRARIES})