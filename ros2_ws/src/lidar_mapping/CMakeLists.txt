cmake_minimum_required(VERSION 3.16)
project(lidar_mapping LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)

find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(lidar_core
    src/core/se3.cpp
    src/core/odometry_buffer.cpp
    src/core/cloud_preprocessor.cpp
    src/core/gicp_registration.cpp
    src/core/map_manager.cpp
    src/core/scan_to_map.cpp
)
target_compile_features(lidar_core PUBLIC cxx_std_17)
target_include_directories(lidar_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_core PUBLIC Eigen3::Eigen pcl_common
    PRIVATE pcl_filters pcl_io pcl_kdtree pcl_registration
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS lidar_core EXPORT export_lidar_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_lidar_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)

add_executable(lidar_mapping_node
    src/ros_integration/lidar_mapping_node.cpp
    src/ros_integration/node_config.cpp
)
target_link_libraries(lidar_mapping_node lidar_core)
ament_target_dependencies(lidar_mapping_node
    rclcpp rcl_interfaces builtin_interfaces sensor_msgs nav_msgs geometry_msgs
    diagnostic_msgs std_srvs tf2_ros pcl_conversions
)
install(TARGETS lidar_mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_test REQUIRED)
    ament_add_gtest(lidar_core_tests test/core_tests.cpp test/cloud_tests.cpp test/map_manager_tests.cpp test/scan_to_map_tests.cpp)
    if(NOT TARGET lidar_core_tests)
        message(FATAL_ERROR "GoogleTest is required with BUILD_TESTING enabled")
    endif()
    target_link_libraries(lidar_core_tests lidar_core)

    add_executable(lidar_synthetic test/synthetic_scan_to_map.cpp)
    target_link_libraries(lidar_synthetic PRIVATE lidar_core)
    ament_add_test(lidar_synthetic
        COMMAND "$<TARGET_FILE:lidar_synthetic>" "${CMAKE_CURRENT_BINARY_DIR}/synthetic"
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        TIMEOUT 180
    )
endif()

ament_package()
