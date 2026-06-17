cmake_minimum_required(VERSION 3.0.2)
project(duckietown_planning)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  src/intersection_navigator_node.py
  src/route_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
