cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
add_compile_options(-std=c++11)




find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)


add_executable(test ./test.cpp)
target_link_libraries(test  ${OpenCV_LIBRARIES} )


