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
project(direct_lidar_inertial_odometry)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePcd.srv"
)

include_directories(include)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2_ros)

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/nano_gicp.cc
  src/nano_gicp/lsq_registration.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
ament_target_dependencies(nano_gicp rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2_ros)

# Odometry Node
add_executable(dlio_odom_node src/dlio/odom_node.cc src/dlio/odom.cc)
target_compile_options(dlio_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlio_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)
ament_target_dependencies(dlio_odom_node
  rclcpp std_msgs sensor_msgs geometry_msgs
  nav_msgs pcl_conversions tf2_ros)

# Mapping Node
add_executable(dlio_map_node src/dlio/map_node.cc src/dlio/map.cc)
target_compile_options(dlio_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlio_map_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
ament_target_dependencies(dlio_map_node
  rclcpp std_msgs sensor_msgs geometry_msgs
  nav_msgs pcl_conversions tf2_ros)

# Link service type support
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dlio_odom_node ${cpp_typesupport_target})
target_link_libraries(dlio_map_node ${cpp_typesupport_target})

# Install
install(TARGETS dlio_odom_node dlio_map_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
