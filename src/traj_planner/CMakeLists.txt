cmake_minimum_required(VERSION 2.8.3)
project(traj_planner)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/arc_utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/sdf_tools)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(NLopt_DIR /usr/local/lib/cmake/nlopt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  sdf_tools
  NLopt
) 
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/sdf_tools/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/arc_utilities/include)
include_directories(${NLopt_INCLUDE_DIRS})

add_executable(traj_generate
  src/traj_generate.cpp
  src/Astar_searcher.cpp
  src/qp_generator.cpp
)
target_link_libraries(traj_generate 
  ${catkin_LIBRARIES}
  ${NLopt_LIBRARIES} 
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/lib/libnlopt.so.0.11.1
)
