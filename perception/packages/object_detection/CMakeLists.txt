cmake_minimum_required(VERSION 3.0.2)
project(object_detection)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS src/object_detection_node.py src/apriltag_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
