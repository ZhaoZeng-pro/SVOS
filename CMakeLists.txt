cmake_minimum_required(VERSION 3.23)
project(SVOS)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#include_directories(
#        /opt/intel/oneapi/mkl/2023.0.0/include/
#        include  #-------即/usr/local/include路径
#)
include_directories(${PCL_INCLUDE_DIRS})

#link_directories(/opt/intel/oneapi/mkl/2023.0.0/lib/intel64
#        )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
aux_source_directory(. ALL_SRCS)
add_executable(SVOS ${ALL_SRCS})

target_link_libraries(SVOS ${PCL_LIBRARIES})
