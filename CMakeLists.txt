cmake_minimum_required(VERSION 2.8)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-std=c++11)

project( AugmSSL )
set(CMAKE_PREFIX_PATH /usr/local/share/OpenCV/)
#set(CMAKE_PREFIX_PATH /usr/share/opencv/)
find_package( OpenCV REQUIRED )
message( "path: ${CMAKE_PREFIX_PATH}" )

find_package( Boost REQUIRED COMPONENTS filesystem system thread )
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(src/AugmSSL)
#add_subdirectory(src/Examples)
