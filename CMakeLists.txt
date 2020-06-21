# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(cloud_converter)

find_package(PCL 1.10 REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
set(THIRD_PARTY_LIBS ${PCL_LIBRARIES})

include_directories(${THIRD_PARTY_INCLUDE_DIRS})

# Declare the executable target built from your sources
add_executable(cloud_converter converter.cpp)

# Link your application with OpenCV libraries
target_link_libraries(cloud_converter PRIVATE ${THIRD_PARTY_LIBS})