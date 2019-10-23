cmake_minimum_required(VERSION 2.8.3)
project(message-practice)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  number_oddeven.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES message_practice
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(Node1 src/node_1.cpp)
add_dependencies(Node1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node1 ${catkin_LIBRARIES})

add_executable(Node2 src/node_2.cpp)
add_dependencies(Node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node2 ${catkin_LIBRARIES})

add_executable(Node3 src/node_3.cpp)
add_dependencies(Node3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node3 ${catkin_LIBRARIES})
