###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(dliom)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_VERSION 17)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL 1.10.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(GTSAM 4.0.3 EXACT REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  message_generation
  backward_ros
)

add_service_files(
  DIRECTORY srv
  FILES
  save_pcd.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    nano_gicp
    nanoflann
)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

set(DLIOM_LIBS
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${OpenMP_LIBS} 
    Threads::Threads 
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    gtsam
    ${GeographicLib_LIBRARIES}
    nano_gicp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${GeographicLib_INCLUDE_DIRS}
) 

# NanoFLANN
add_library(nanoflann STATIC src/nano_gicp/nanoflann.cc)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_compile_options(nanoflann PRIVATE -Wall -Wextra -Wpedantic)

# NanoGICP
add_library(nano_gicp STATIC src/nano_gicp/lsq_registration.cc src/nano_gicp/nano_gicp.cc)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX nanoflann)
target_compile_options(nano_gicp PRIVATE -Wall -Wextra -Wpedantic)

# Odometry Node
add_executable(dliom_odom_node src/dliom/odom_node.cc src/dliom/odom.cc)
add_dependencies(dliom_odom_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(dliom_odom_node ${DLIOM_LIBS})
target_compile_options(dliom_odom_node PRIVATE -Wall -Wextra -Wpedantic)

# Mapping Node
add_executable(dliom_map_node src/dliom/map_node.cc src/dliom/map.cc)
add_dependencies(dliom_map_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(dliom_map_node ${DLIOM_LIBS})
target_compile_options(dliom_map_node PRIVATE -Wall -Wextra -Wpedantic)

# GPS Debug Node
add_executable(gps_debug_node src/misc/gps_debug.cc)
target_link_libraries(gps_debug_node ${DLIOM_LIBS})
target_compile_options(gps_debug_node PRIVATE -Wall -Wextra -Wpedantic)

# Binaries
install(TARGETS dliom_odom_node dliom_map_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install(DIRECTORY cfg launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
