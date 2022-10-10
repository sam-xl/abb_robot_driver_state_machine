cmake_minimum_required(VERSION 3.0.2)
project(abb_robot_driver_state_machine)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS src/state_machine_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
