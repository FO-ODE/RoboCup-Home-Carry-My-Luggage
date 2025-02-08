cmake_minimum_required(VERSION 3.0.2)
project(carry_task_manager)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  move_base_msgs
  smach
  smach_ros
)


catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib move_base_msgs smach smach_ros
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/task_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


