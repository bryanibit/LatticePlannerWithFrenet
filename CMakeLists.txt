cmake_minimum_required(VERSION 3.2)
project(frenet)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -o3 -pthread")
include_directories(/usr/include/eigen3)
include_directories(include)

find_package(OpenCV 3.3.1 REQUIRED)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory("./src" SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
