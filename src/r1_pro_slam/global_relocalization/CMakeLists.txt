cmake_minimum_required(VERSION 3.5)
project(indoor_location)

# 设置 C++14（你可以根据需求修改为 C++17）
set(CMAKE_CXX_STANDARD 17)

# 设置 CMP0074 策略以消除警告
cmake_policy(SET CMP0074 NEW)

# 查找 ROS 2 的依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(fmt REQUIRED)

set(_sophus_local_include "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/Sophus/install/include")
set(_ndt_omp_local_include "${CMAKE_CURRENT_SOURCE_DIR}/../ndt_omp_ros2/include")
set(_ndt_omp_local_src "${CMAKE_CURRENT_SOURCE_DIR}/../ndt_omp_ros2/src")
if(EXISTS "${_sophus_local_include}/sophus/se3.hpp")
  message(STATUS "Using local Sophus headers from ${_sophus_local_include}")
else()
  find_package(Sophus REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# 包含目录
include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # 添加生成的接口目录
        src
        src/nano_flann
        src/utility
        src/cloud_process/livox_mid360_lidar  #livox_mid360_lidar 雷达
        src/cloud_process/rs16_lidar #rs16_lidar 雷达
        src/p2pl_icp
        src/eskf
        src/pose_persistence
        src/localization_state
        src/global_relocalization

        include
        ${rclcpp_INCLUDE_DIRS}
        ${rosidl_generate_interfaces_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}

        ${std_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${_sophus_local_include}
        ${_ndt_omp_local_include}
        /usr/include/eigen3  # 如果 EIGEN3_INCLUDE_DIRS 未正确设置，可以手动指定路径
)

# NanoFLANN 库
add_library(nanoflann STATIC
        src/nano_flann/nanoflann.cc
        )
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

add_executable(location_node
        src/location_p2pl_icp.cpp
        src/cloud_process/livox_mid360_lidar/cloud_process_livox.cpp #livox 数据处理
        src/cloud_process/rs16_lidar/cloud_process_rs.cpp  #rs 速腾16线数据处理
        src/global_relocalization/bbs_localization.cpp
        src/global_relocalization/global_relocalization.cpp
        src/localization_state/localization_state.cpp
        src/p2pl_icp/p2pl_icp.cpp
        src/pose_persistence/pose_persistence.cpp
        src/pose_persistence/pose_save_policy.cpp
        src/utility/utility.cpp
        src/eskf/eskf.cpp
        ${_ndt_omp_local_src}/pclomp/voxel_grid_covariance_omp.cpp
        ${_ndt_omp_local_src}/pclomp/ndt_omp.cpp
        )
ament_target_dependencies(location_node
        rclcpp
        PCL
        builtin_interfaces
        pcl_conversions
        yaml-cpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav_msgs
        )
target_link_libraries(location_node
        nanoflann yaml-cpp
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        fmt::fmt
        "${cpp_typesupport_target}"
        $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
        )

if(NOT EXISTS "${_sophus_local_include}/sophus/se3.hpp")
  target_link_libraries(location_node Sophus::Sophus)
endif()

# 安装目标 (独立的点云处理节点已删除，功能已集成到 location_node)
install(TARGETS
        location_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# 安装包含目录
install(DIRECTORY "include/"
        DESTINATION include
        )

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  ament_add_gtest(test_pose_persistence
    src/utility/test_pose_persistence.cpp
  )
  if(TARGET test_pose_persistence)
    target_include_directories(test_pose_persistence PUBLIC include)
    target_sources(test_pose_persistence PRIVATE
      src/pose_persistence/pose_persistence.cpp
      src/utility/utility.cpp
    )
    ament_target_dependencies(test_pose_persistence
      geometry_msgs
      nav_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
    )
    target_link_libraries(test_pose_persistence
      yaml-cpp
      ${PCL_LIBRARIES}
      fmt::fmt
      $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
    )
  endif()

  ament_add_gtest(test_global_relocalization
    src/p2pl_icp/test_global_relocalization.cpp
  )
  if(TARGET test_global_relocalization)
    target_include_directories(test_global_relocalization PUBLIC include)
    target_sources(test_global_relocalization PRIVATE
      src/global_relocalization/bbs_localization.cpp
      src/global_relocalization/global_relocalization.cpp
      src/utility/utility.cpp
    )
    ament_target_dependencies(test_global_relocalization
      geometry_msgs
      nav_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
    )
    target_link_libraries(test_global_relocalization
      nanoflann
      yaml-cpp
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      fmt::fmt
      $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
    )
  endif()

  ament_add_gtest(test_pose_save_policy
    src/utility/test_pose_save_policy.cpp
  )
  if(TARGET test_pose_save_policy)
    target_include_directories(test_pose_save_policy PUBLIC include)
    target_sources(test_pose_save_policy PRIVATE
      src/pose_persistence/pose_save_policy.cpp
      src/utility/utility.cpp
    )
    ament_target_dependencies(test_pose_save_policy
      geometry_msgs
      nav_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
    )
    target_link_libraries(test_pose_save_policy
      yaml-cpp
      ${PCL_LIBRARIES}
      fmt::fmt
      $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
    )
  endif()
endif()

ament_package()
