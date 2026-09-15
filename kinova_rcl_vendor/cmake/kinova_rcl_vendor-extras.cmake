# Exported by find_package(kinova_rcl_vendor).
#
# Provides:
#   KINOVA_RCL_INCLUDE_DIR  - include dir for Kinova's <rcl/...> headers
#   kinova_rcl::rcl         - imported STATIC target for librcl.a
#
# IMPORTANT: kinova_rcl::rcl carries NO INTERFACE_INCLUDE_DIRECTORIES, by design.
# Kinova's <rcl/rcl.h> collides with ROS 2's rcl package, which rclcpp pulls in
# transitively, and the two cannot coexist in one translation unit under any
# include order. Add ${KINOVA_RCL_INCLUDE_DIR} by hand, and only to a target
# isolated from rclcpp.

# ${kinova_rcl_vendor_DIR} is <prefix>/share/kinova_rcl_vendor/cmake
get_filename_component(_kinova_rcl_prefix "${kinova_rcl_vendor_DIR}/../../.." ABSOLUTE)
set(_kinova_rcl_root "${_kinova_rcl_prefix}/opt/kinova_rcl")

set(KINOVA_RCL_INCLUDE_DIR "${_kinova_rcl_root}/include")
set(KINOVA_RCL_LIBRARY "${_kinova_rcl_root}/lib/librcl.a")

if(NOT EXISTS "${KINOVA_RCL_LIBRARY}")
  message(FATAL_ERROR
    "kinova_rcl_vendor is installed but ${KINOVA_RCL_LIBRARY} is missing. "
    "Rebuild kinova_rcl_vendor.")
endif()

if(NOT TARGET kinova_rcl::rcl)
  add_library(kinova_rcl::rcl STATIC IMPORTED)
  set_target_properties(kinova_rcl::rcl PROPERTIES
    IMPORTED_LOCATION "${KINOVA_RCL_LIBRARY}"
    # RCL spawns real-time threads and dlopen()s its EtherCAT master backends.
    INTERFACE_LINK_LIBRARIES "${CMAKE_DL_LIBS};pthread"
  )
endif()

unset(_kinova_rcl_prefix)
unset(_kinova_rcl_root)
