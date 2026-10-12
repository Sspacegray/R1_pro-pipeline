cmake_minimum_required(VERSION 3.8)
project(r1pro_aruco_box_pose)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/aruco_box_pose.py
  scripts/detect_aruco_image.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
