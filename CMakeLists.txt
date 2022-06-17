cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME realsense_t265_interface)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  mavros_msgs
  mav_msgs
  nav_msgs
  robot_process
  tf
  tf2
  ros_utils_lib
)

catkin_package(
  INCLUDE_DIRS src/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs mavros_msgs robot_process mav_msgs tf ros_utils_lib nav_msgs tf2
  )

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)


set(CPP_PROJECT_FILES 
  src/source/realsense_t265_interface.cpp
)

set(HEADER_PROJECT_FILES
  src/include/realsense_t265_interface.hpp
)

# add_library(${PROJECT_NAME}
#   ${CPP_PROJECT_FILES}
#   ${HEADER_PROJECT_FILES}
# )
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/source/realsense_t265_interface_node.cpp  ${CPP_PROJECT_FILES} ${HEADER_PROJECT_FILES}) 
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})