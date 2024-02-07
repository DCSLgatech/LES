cmake_minimum_required(VERSION 2.8.3)
project(trust_region)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  rospy  
  std_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES relevant_region
CATKIN_DEPENDS 
   roscpp
   rospy
   std_msgs   
   moveit_core
   moveit_visual_tools
   moveit_ros_planning_interface
   interactive_markers
#  DEPENDS system_lib
DEPENDS
    EIGEN3
)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIR}  ${OMPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(src)
link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########
SET( CMAKE_CXX_FLAGS  "-std=c++0x" )
SET( CMAKE_BUILD_TYPE Release)

# RRTsharp Library 
add_library(${PROJECT_NAME}_rrtsharp
  src/rrtsharp/MRRTsharp.cpp src/rrtsharp/MRRTXstatic.cpp
)
target_link_libraries(${PROJECT_NAME}_rrtsharp
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# ESTRELRRTsharp Library 
add_library(${PROJECT_NAME}_estrelrrtsharp
  src/estrelrrtsharp/ESTRELRRTsharp.cpp src/estrelrrtsharp/ESTRELRRTXstatic.cpp
)
target_link_libraries(${PROJECT_NAME}_estrelrrtsharp
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# DRRT Library 
add_library(${PROJECT_NAME}_drrt
  src/drrt/DRRT.cpp 
)
target_link_libraries(${PROJECT_NAME}_drrt
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# LESDRRT Library 
add_library(${PROJECT_NAME}_lesdrrt
  src/lesdrrt/LESDRRT.cpp 
)
target_link_libraries(${PROJECT_NAME}_lesdrrt
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# TRUSTRRTsharp Library 
add_library(${PROJECT_NAME}_trustrrtsharp
  src/trustrrtsharp/TRUSTRRTsharp.cpp src/trustrrtsharp/TRUSTRRTXstatic.cpp
)
target_link_libraries(${PROJECT_NAME}_trustrrtsharp
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# OMPLtools Library 
add_library(${PROJECT_NAME}_ompltools
  src/utilities/OMPLtools.cpp
)
target_link_libraries(${PROJECT_NAME}_ompltools
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES}
)

# PPM Library for loading images
add_library(${PROJECT_NAME}_ppm
  src/utilities/ppm.cpp
)
target_link_libraries(${PROJECT_NAME}_ppm
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

## Declare a C++ executable
add_executable(trustregion_visualize src/trustregion_visualize.cpp)
target_link_libraries(trustregion_visualize 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_rrtsharp  
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
  ${PROJECT_NAME}_ompltools
)

add_executable(trustregion_visualize_6d src/trustregion_visualize_6d.cpp)
target_link_libraries(trustregion_visualize_6d 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_rrtsharp  
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
  ${PROJECT_NAME}_ompltools
)

add_executable(trustregion_costmap_visualize src/trustregion_costmap_visualize.cpp   )
target_link_libraries(trustregion_costmap_visualize
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_ppm
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
  ${PROJECT_NAME}_ompltools
)

add_executable(trustregion_costmap_visualize_4d src/trustregion_costmap_visualize_4d.cpp   )
target_link_libraries(trustregion_costmap_visualize_4d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_ppm
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
  ${PROJECT_NAME}_ompltools
)

add_executable(trustregion_costmap_visualize_6d src/trustregion_costmap_visualize_6d.cpp   )
target_link_libraries(trustregion_costmap_visualize_6d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_ppm
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
  ${PROJECT_NAME}_ompltools
)

add_executable(trustregion_panda_visualize src/trustregion_panda_visualize.cpp)
target_link_libraries(trustregion_panda_visualize
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}   
  ${OMPL_LIBRARIES}
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_baxter_visualize src/trustregion_baxter_visualize.cpp)
target_link_libraries(trustregion_baxter_visualize
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}   
  ${OMPL_LIBRARIES}
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

## benchmarking nodes 
add_executable(trustregion_benchmark_panda src/benchmarking/panda/trustregion_benchmark_panda.cpp   )
target_link_libraries(trustregion_benchmark_panda
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_panda_mani src/benchmarking/panda_mani/trustregion_benchmark_panda_mani.cpp   )
target_link_libraries(trustregion_benchmark_panda_mani
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_baxter src/benchmarking/baxter/trustregion_benchmark_baxter.cpp   )
target_link_libraries(trustregion_benchmark_baxter
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_potential2d src/benchmarking/potential2d/trustregion_benchmark_potential2d.cpp   )
target_link_libraries(trustregion_benchmark_potential2d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_potential4d src/benchmarking/potential4d/trustregion_benchmark_potential4d.cpp   )
target_link_libraries(trustregion_benchmark_potential4d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_potential6d src/benchmarking/potential6d/trustregion_benchmark_potential6d.cpp   )
target_link_libraries(trustregion_benchmark_potential6d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_lesdrrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_potential8d src/benchmarking/potential8d/trustregion_benchmark_potential8d.cpp   )
target_link_libraries(trustregion_benchmark_potential8d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_2d src/benchmarking/2d/trustregion_benchmark_2d.cpp   )
target_link_libraries(trustregion_benchmark_2d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_4d src/benchmarking/4d/trustregion_benchmark_4d.cpp   )
target_link_libraries(trustregion_benchmark_4d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

add_executable(trustregion_benchmark_6d src/benchmarking/6d/trustregion_benchmark_6d.cpp   )
target_link_libraries(trustregion_benchmark_6d
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${PROJECT_NAME}_rrtsharp
  ${PROJECT_NAME}_drrt
  ${PROJECT_NAME}_trustrrtsharp
  ${PROJECT_NAME}_estrelrrtsharp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
