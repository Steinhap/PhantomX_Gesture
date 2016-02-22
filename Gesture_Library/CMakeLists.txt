cmake_minimum_required(VERSION 2.8.3)
project(Gesture_Library)

find_package(catkin REQUIRED COMPONENTS
  arbotix_controllers
  arbotix_python
)
catkin_python_setup()

find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

