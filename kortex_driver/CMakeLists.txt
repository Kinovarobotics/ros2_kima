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

# --- Kinova Robot Control Library (RCL) SDK ---------------------------------
# Not a ROS package. Point KINOVA_RCL_ROOT at the extracted SDK release
# directory, or override with -DKINOVA_RCL_ROOT=...
set(KINOVA_RCL_ROOT
  "/home/aalmrad/Documents/KIMA/RCL/robot_control_library_linux_x86_gcc13_release_0.1.0-dev.126"
  CACHE PATH "Path to the extracted Kinova RCL SDK (contains include/ and lib/).")

# IMPORTANT: link the STATIC archive (librcl.a), not librcl.so. Kinova's shared
# library has the SONAME `librcl.so`, which is identical to ROS 2's own `rcl`
# library — the dynamic loader would resolve our `librcl.so` NEEDED entry to
# ROS's librcl.so and leave every Kinova rcl::api symbol undefined. Statically
# linking the archive folds the Kinova code into this plugin and removes the
# runtime dependency (and the name clash) altogether.
set(RCL_LIBRARY "${KINOVA_RCL_ROOT}/lib/librcl.a")
if(NOT EXISTS "${RCL_LIBRARY}")
  message(FATAL_ERROR
    "librcl.a not found at ${RCL_LIBRARY}. Set -DKINOVA_RCL_ROOT to the RCL SDK path.")
endif()

# Imported handle for librcl.a. NOTE: deliberately NO INTERFACE_INCLUDE_DIRECTORIES
# — Kinova's <rcl/rcl.h> collides with ROS 2's rcl, so the SDK include dir must
# never leak onto a target that also pulls in rclcpp.
add_library(rcl_sdk STATIC IMPORTED)
set_target_properties(rcl_sdk PROPERTIES IMPORTED_LOCATION "${RCL_LIBRARY}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

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
    "${KINOVA_RCL_ROOT}/include"
)
# PUBLIC so librcl.so propagates to the plugin for linking; rcl_sdk exposes no
# include dirs, so nothing leaks.
target_link_libraries(kortex_rcl_driver PUBLIC rcl_sdk ${CMAKE_DL_LIBS} pthread)
set_target_properties(kortex_rcl_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)

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
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
