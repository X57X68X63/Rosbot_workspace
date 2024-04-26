# Install script for directory: /home/haocheng/Rosbot_workspace/src/explore

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/haocheng/Rosbot_workspace/install/explore_lite")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/explore/" TYPE DIRECTORY FILES "/home/haocheng/Rosbot_workspace/src/explore/include/explore/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE DIRECTORY FILES "/home/haocheng/Rosbot_workspace/src/explore/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE DIRECTORY FILES "/home/haocheng/Rosbot_workspace/src/explore/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/explore_lite" TYPE EXECUTABLE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/explore")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore"
         OLD_RPATH "/home/haocheng/ros2_humble/install/ament_index_cpp/lib:/home/haocheng/ros2_humble/install/builtin_interfaces/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/haocheng/ros2_humble/install/rmw/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/haocheng/ros2_humble/install/rcutils/lib:/home/haocheng/ros2_humble/install/rcpputils/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_c/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/haocheng/ros2_humble/install/rosidl_runtime_c/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/haocheng/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/haocheng/ros2_humble/install/std_msgs/lib:/home/haocheng/ros2_humble/install/libstatistics_collector/lib:/home/haocheng/ros2_humble/install/rcl/lib:/home/haocheng/ros2_humble/install/rosgraph_msgs/lib:/home/haocheng/ros2_humble/install/rcl_yaml_param_parser/lib:/home/haocheng/ros2_humble/install/statistics_msgs/lib:/home/haocheng/ros2_humble/install/tracetools/lib:/home/haocheng/ros2_humble/install/tf2_ros/lib:/home/haocheng/ros2_humble/install/map_msgs/lib:/home/haocheng/ros2_humble/install/nav_msgs/lib:/home/haocheng/ros2_humble/install/visualization_msgs/lib:/home/haocheng/ros2_humble/install/message_filters/lib:/home/haocheng/ros2_humble/install/rclcpp_action/lib:/home/haocheng/ros2_humble/install/tf2_msgs/lib:/home/haocheng/ros2_humble/install/laser_geometry/lib:/home/haocheng/ros2_humble/install/sensor_msgs/lib:/home/haocheng/ros2_humble/install/tf2/lib:/home/haocheng/ros2_humble/install/geometry_msgs/lib:/home/haocheng/ros2_humble/install/class_loader/lib:/home/haocheng/ros2_humble/install/rclcpp_lifecycle/lib:/home/haocheng/ros2_humble/install/std_srvs/lib:/home/haocheng/ros2_humble/install/rcl_action/lib:/home/haocheng/ros2_humble/install/action_msgs/lib:/home/haocheng/ros2_humble/install/unique_identifier_msgs/lib:/home/haocheng/ros2_humble/install/rcl_lifecycle/lib:/home/haocheng/ros2_humble/install/rcl_interfaces/lib:/home/haocheng/ros2_humble/install/libyaml_vendor/lib:/home/haocheng/ros2_humble/install/rmw_implementation/lib:/home/haocheng/ros2_humble/install/rcl_logging_spdlog/lib:/home/haocheng/ros2_humble/install/rcl_logging_interface/lib:/home/haocheng/ros2_humble/install/lifecycle_msgs/lib:/home/haocheng/ros2_humble/install/fastcdr/lib:/opt/ros/humble/lib:/home/haocheng/ros2_humble/install/rclcpp/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/explore_lite/explore")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/explore_lite")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/explore_lite")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/environment" TYPE FILE FILES "/home/haocheng/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/environment" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/environment" TYPE FILE FILES "/home/haocheng/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/environment" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_index/share/ament_index/resource_index/packages/explore_lite")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/cmake" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite/cmake" TYPE FILE FILES
    "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_core/explore_liteConfig.cmake"
    "/home/haocheng/Rosbot_workspace/build/explore_lite/ament_cmake_core/explore_liteConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/explore_lite" TYPE FILE FILES "/home/haocheng/Rosbot_workspace/src/explore/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/haocheng/Rosbot_workspace/build/explore_lite/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/haocheng/Rosbot_workspace/build/explore_lite/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
