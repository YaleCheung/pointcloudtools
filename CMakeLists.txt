cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PCLTools)

#  use c++17 library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g")

# eigen import
find_package(Eigen3 REQUIRED )
include_directories( EIGEN3_INCLUDE_DIR )

# pcl import
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# boost import
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost COMPONENTS thread filesystem)
include_directories(${Boost_INCLUDE_DIRS})

# local directory
include_directories(include)
include_directories(deps/cxxopts/include)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(LINK_LIBS ${PCL_LIBRARIES}
              ${Boost_LIBRARIES}
              ${OpenCV_LIBS})

add_subdirectory(src)
add_subdirectory(test)
