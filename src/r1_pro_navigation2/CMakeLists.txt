cmake_minimum_required(VERSION 3.8)
project(r1_pro_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_cmd_vel_safety
    test/test_cmd_vel_safety.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_navigation_health_gate
    test/test_navigation_health_gate.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_lidar_self_filter
    test/test_lidar_self_filter.py
    TIMEOUT 60
  )
endif()


install(DIRECTORY launch params bts rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_to_twist_stamped.py
  scripts/navigation_preflight.py
  scripts/web_navigation_status.py
  scripts/goal_pose_stamp_filter.py
  scripts/path_frame_normalizer.py
  scripts/navigation_status_monitor.py
  scripts/call_global_localization.py
  scripts/publish_initial_pose.py
  scripts/lidar_self_filter.py
  scripts/wait_for_tf_ready.py
  scripts/print_bringup_banner.py
  scripts/robot_tf_watchdog.py
  scripts/robot_description_publisher.py
  scripts/start_r1pro_base.sh
  scripts/start_r1pro_navigation.sh
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
