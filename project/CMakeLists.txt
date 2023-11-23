cmake_minimum_required(VERSION 3.0.2)
project(project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  mavros_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Drone_pos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES project
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime
)

add_executable(startcpp_single src/startcpp_single.cpp)
add_executable(drone_controller src/drone_controller.cpp)
add_executable(drone1_controller src/drone1_controller.cpp)
add_executable(drone2_controller src/drone2_controller.cpp)
add_executable(drone3_controller src/drone3_controller.cpp)
add_executable(collision_avoidance src/collision_avoidance.cpp)
add_executable(drone_tracker src/drone_tracker.cpp)

include_directories( include ${catkin_INCLUDE_DIRS} )

target_link_libraries( startcpp_single ${catkin_LIBRARIES} )
target_link_libraries( drone_controller ${catkin_LIBRARIES} )
target_link_libraries( drone1_controller ${catkin_LIBRARIES} )
target_link_libraries( drone2_controller ${catkin_LIBRARIES} )
target_link_libraries( drone3_controller ${catkin_LIBRARIES} )
target_link_libraries( collision_avoidance ${catkin_LIBRARIES} )
target_link_libraries( drone_tracker ${catkin_LIBRARIES} )

add_dependencies(drone_controller project_generate_messages_cpp)
add_dependencies(drone1_controller project_generate_messages_cpp)
add_dependencies(drone2_controller project_generate_messages_cpp)
add_dependencies(drone3_controller project_generate_messages_cpp)
add_dependencies(collision_avoidance project_generate_messages_cpp)
