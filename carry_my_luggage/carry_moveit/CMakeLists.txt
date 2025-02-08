cmake_minimum_required(VERSION 2.8.3)
project(carry_moveit)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  geometry_msgs
  actionlib
  move_base_msgs
  teleop_tools_msgs
)

catkin_package()


include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_executable(arm_torso src/arm_torso.cpp)
add_executable(arm src/arm.cpp)
add_executable(grasp src/grasp.cpp)
add_executable(drop src/drop.cpp)


add_dependencies(arm_torso ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(arm_torso ${catkin_LIBRARIES})
target_link_libraries(arm ${catkin_LIBRARIES})
target_link_libraries(grasp ${catkin_LIBRARIES})
target_link_libraries(drop ${catkin_LIBRARIES})


install(TARGETS arm_torso arm grasp drop
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/tuck_arm.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_install_python(PROGRAMS
  scripts/tuck_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)