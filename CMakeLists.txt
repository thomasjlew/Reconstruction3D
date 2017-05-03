cmake_minimum_required(VERSION 2.8)
project(cvision)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(cvision
	src/main.cpp
	src/features.cpp
	src/utils.cpp
)

set_target_properties(cvision PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3")
target_link_libraries(cvision ${OpenCV_LIBS})
