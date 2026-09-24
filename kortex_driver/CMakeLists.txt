cmake_minimum_required(VERSION 3.14)
project(kortex_driver)

# RCL requires C++20 and the gcc-13 (new __cxx11) ABI.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# --- Kinova Robot Control Library (RCL) SDK ---------------------------------
# Not a ROS package. Vendored by kinova_rcl_vendor, which downloads the released
# tarball and exports the imported target kinova_rcl::rcl plus the variable
# KINOVA_RCL_INCLUDE_DIR.
#
# IMPORTANT: that target links the STATIC archive (librcl.a), not librcl.so, and
# carries no interface include directories. Kinova's shared library has the
# SONAME `librcl.so`, identical to ROS 2's own `rcl` library — the dynamic loader
# would resolve our `librcl.so` NEEDED entry to ROS's and leave every Kinova
# rcl::api symbol undefined. Its headers collide for the same reason, so
# KINOVA_RCL_INCLUDE_DIR must never be added to a target that also pulls in
# rclcpp. Static linking folds the Kinova code into this plugin and removes the
# runtime dependency (and the name clash) altogether.
find_package(kinova_rcl_vendor REQUIRED)

# --- ROS-free RCL wrapper ---------------------------------------------------
# Compiled in isolation: this is the only target that sees the Kinova RCL
# include directory, and it deliberately does NOT depend on rclcpp/hardware_interface,
# so ROS 2's rcl headers never share a translation unit with Kinova's.
add_library(kortex_rcl_driver STATIC
  src/rcl_robot_driver.cpp
)
target_include_directories(kortex_rcl_driver
  PRIVATE
    include
    "${KINOVA_RCL_INCLUDE_DIR}"
)
# PUBLIC so librcl.a propagates to the plugin for linking; kinova_rcl::rcl
# exposes no include dirs, so nothing leaks.
target_link_libraries(kortex_rcl_driver PUBLIC kinova_rcl::rcl)
set_target_properties(kortex_rcl_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Standalone fault-clear tool --------------------------------------------
# ROS-free for the same reason as kortex_rcl_driver: it includes Kinova's
# <rcl/rcl.h>. Run with `ros2 run kortex_driver clear_faults <topology.yaml>`.
add_executable(clear_faults tools/clear_faults.cpp)
target_include_directories(clear_faults PRIVATE "${KINOVA_RCL_INCLUDE_DIR}")
target_link_libraries(clear_faults kinova_rcl::rcl)

# --- ros2_control hardware plugin -------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
  src/kortex_math_util.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)
target_link_libraries(${PROJECT_NAME} kortex_rcl_driver)
ament_target_dependencies(
  ${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
)
# RCL is statically linked (see above), so there is no librcl.so runtime
# dependency and no RPATH is needed for it.

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS clear_faults
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
