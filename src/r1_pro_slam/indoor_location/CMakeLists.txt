cmake_minimum_required(VERSION 3.5)
project(indoor_location)

# 默认 Release (-O3): ICP/ESKF 数值密集, 无优化(-O0)时单帧耗时250ms+跟不上10Hz
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# 设置 C++14（你可以根据需求修改为 C++17）
set(CMAKE_CXX_STANDARD 17)

# 设置 CMP0074 策略以消除警告
cmake_policy(SET CMP0074 NEW)

# 查找 ROS 2 的依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)


# 包含目录
include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  # 添加生成的接口目录
        src/nano_flann
        src/utility
        src/cloud_process/livox_mid360_lidar  #livox_mid360_lidar 雷达
        src/cloud_process/rs16_lidar #rs16_lidar 雷达
        src/p2pl_icp
        src/eskf

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
        src/p2pl_icp/p2pl_icp.cpp
        src/utility/utility.cpp
        src/eskf/eskf.cpp
        )
ament_target_dependencies(location_node
        rclcpp
        PCL
        pcl_conversions
        yaml-cpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav_msgs
        )
target_link_libraries(location_node
        nanoflann yaml-cpp
        Sophus::Sophus
        fmt::fmt
        "${cpp_typesupport_target}"
        $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
        )

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

ament_package()