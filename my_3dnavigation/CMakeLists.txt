cmake_minimum_required(VERSION 2.4.6)
project(my_3dnavigation)

# c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# include cmake files in the 'cmake folder'
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_sensor_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_3dnavigation
#  CATKIN_DEPENDS roscpp  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/my_3dnavigation
  /home/gary/Desktop/eigen-eigen-07105f7124f9
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(my_3dnavigation
#   src/${PROJECT_NAME}/my_3dnavigation.cpp
# )

# libRRT
################################################################################

set(lib_SRC
    "src/3dplane/3dplane.cpp"
    "src/3dplane/GridStateSpace3d.cpp"
    "src/3dplane/ObstacleGrid3d.cpp"
    "src/3dplane/PlaneStateSpace3d.cpp"
)

add_library(${PROJECT_NAME}
    src/3dplane/3dplane.cpp
    src/3dplane/GridStateSpace3d.cpp
    src/3dplane/ObstacleGrid3d.cpp
    src/3dplane/PlaneStateSpace3d.cpp

)

## Declare a C++ executable
# add_executable(my_3dnavigation_node src/my_3dnavigation_node.cpp)
add_executable(my_3dnavigationtest_TestEigen src/rrtplanner/TestEigen.cpp)
add_executable(my_3dnavigationtest_Mixnode src/rrtplanner/MixNode.cpp)
add_executable(my_3dnavigationtest_DrawCurveTest src/rrtplanner/DrawCurveTest.cpp)
add_executable(my_3dnavigationtest_SendPoints src/rrtplanner/SendPoints.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(my_3dnavigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(my_3dnavigationtest_TestEigen ${catkin_LIBRARIES})
target_link_libraries(my_3dnavigationtest_Mixnode ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(my_3dnavigationtest_DrawCurveTest ${catkin_LIBRARIES})
target_link_libraries(my_3dnavigationtest_SendPoints ${catkin_LIBRARIES})



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

catkin_add_gtest(${PROJECT_NAME}-test test/birrttest.cpp)
#    if(TARGET ${PROJECT_NAME}-test)
#        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)









