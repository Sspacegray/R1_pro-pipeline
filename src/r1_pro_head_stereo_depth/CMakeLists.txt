cmake_minimum_required(VERSION 3.8)
project(r1_pro_head_stereo_depth)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/head_stereo_depth_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
