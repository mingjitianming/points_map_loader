cmake_minimum_required(VERSION 2.8)
project(map_loader)

set(CMAKE_CXX_FLAGS -std=c++11)

#添加调试模式
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure 
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

#######################################################################
# generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME} 
      src/map_loader_node.cpp
      src/map_loader.cpp
)

add_executable(map_loader_node src/map_loader_node.cpp)
target_link_libraries(map_loader_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(estimate src/estimate.cpp)
target_link_libraries(estimate ${PROJECT_NAME} ${LINK_LIBS})

add_executable(statistical_remove src/statistical_remove.cpp)
target_link_libraries(statistical_remove ${PROJECT_NAME} ${LINK_LIBS})

add_executable(transform_points_map src/transform_points_map.cpp)
target_link_libraries(transform_points_map ${PROJECT_NAME} ${LINK_LIBS})

#add_executable(octomap_color_server_node src/octomap_color_server_node.cpp)
#target_link_libraries(octomap_color_server_node ${PROJECT_NAME} ${LINK_LIBS})
#[[
add_executable(octomap_server_static src/octomap_server_static.cpp)
target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_saver src/octomap_saver.cpp)
target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS}) 

add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
  octomap_server_node
  octomap_server_static
  octomap_server_multilayer
  octomap_saver
  octomap_tracking_server_node
  octomap_server_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})  ]]
