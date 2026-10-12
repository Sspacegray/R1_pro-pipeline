cmake_minimum_required(VERSION 3.8)
project(r1_pro_odom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calibration_tf_publisher.py
  scripts/imu_to_imu_with_cov.py
  scripts/localization_to_odom.py
  scripts/odom_watchdog.py
  scripts/twist_to_twist_with_cov.py
  scripts/wait_for_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
