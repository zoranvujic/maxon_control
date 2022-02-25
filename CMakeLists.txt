cmake_minimum_required(VERSION 3.0.2)
project(maxon_control)


find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    roscpp
    std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Parameters.cfg
)

catkin_package(
 INCLUDE_DIRS 
    include
#  LIBRARIES rosTest
 CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
#  DEPENDS system_lib
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}
	src/maxon_control_node.cpp
	src/speed_control.cpp	
)


target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${std_msgs_LIBRARIES}
 )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

