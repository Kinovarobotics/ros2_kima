cmake_minimum_required(VERSION 3.14)
project(kinova_rcl_vendor)

find_package(ament_cmake REQUIRED)

include(FetchContent)

# --- Released SDK tarball ----------------------------------------------------
# Bump all three together when moving to a new RCL build. Get the hash with:
#   curl -sSL <url> | sha256sum
set(KINOVA_RCL_VERSION "0.1.0-dev.127"
  CACHE STRING "Kinova RCL release to vendor.")
set(KINOVA_RCL_URL
  "https://artifactory.kinovaapps.com/artifactory/generic-local-public/kima/robot_control_library/development/0.1.0-dev/127/robot_control_library_linux_x86_gcc13_release_0.1.0-dev.127.tar.gz"
  CACHE STRING "Download URL for the Kinova RCL release tarball.")
set(KINOVA_RCL_SHA256
  "a48a0f8d97c6cb1bb2d67bdef41f2473dcf426dfba29f77bf6632b8f2c001612"
  CACHE STRING "Expected SHA256 of the Kinova RCL release tarball.")

# The published archive is built for Linux x86_64 against the gcc-13 (new
# __cxx11) ABI. No other platform build is available, so fail loudly rather
# than produce a driver that cannot link.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(FATAL_ERROR
    "Kinova RCL is only published for Linux x86_64 (gcc-13 ABI). "
    "Detected ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}.")
endif()

# CMP0135: use download time, not archive time, for extracted file timestamps —
# otherwise a re-download does not retrigger dependent builds.
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(
  kinova_rcl
  URL ${KINOVA_RCL_URL}
  URL_HASH SHA256=${KINOVA_RCL_SHA256}
)
FetchContent_MakeAvailable(kinova_rcl)

# The tarball has no top-level wrapper directory: include/, lib/, docs/, src/
# and the EULA sit directly in ${kinova_rcl_SOURCE_DIR}.
set(_rcl_include "${kinova_rcl_SOURCE_DIR}/include")
set(_rcl_archive "${kinova_rcl_SOURCE_DIR}/lib/librcl.a")
foreach(_required "${_rcl_include}/rcl/rcl.h" "${_rcl_archive}")
  if(NOT EXISTS "${_required}")
    message(FATAL_ERROR "Unexpected RCL tarball layout: ${_required} is missing.")
  endif()
endforeach()

# --- Install -----------------------------------------------------------------
# Everything goes under opt/kinova_rcl, NOT the ament-standard include/ and lib/.
# Kinova's <rcl/rcl.h> and ROS 2's own rcl package share header paths and a
# SONAME, so these files must never land anywhere ament or the dynamic loader
# would pick up implicitly. Consumers opt in via KINOVA_RCL_INCLUDE_DIR.
install(
  DIRECTORY "${_rcl_include}/"
  DESTINATION opt/kinova_rcl/include
)
# Only the static archive is installed. librcl.so is deliberately dropped: its
# SONAME is librcl.so, identical to ROS 2's, so the loader would resolve one for
# the other at runtime. Consumers static-link the archive instead.
install(
  FILES "${_rcl_archive}"
  DESTINATION opt/kinova_rcl/lib
)

# The EtherCAT network topology descriptions ship in the SDK's examples tree but
# are runtime configuration, not sample code: kortex_driver's
# `network_topology_file` hardware parameter points at one of them. They are
# plain data with no header/SONAME clash, so unlike the headers and archive they
# go to the ament-standard share/ location and are discoverable via the ament
# index.
set(_rcl_topology "${kinova_rcl_SOURCE_DIR}/src/examples/network_topology")
if(NOT IS_DIRECTORY "${_rcl_topology}")
  message(FATAL_ERROR "Unexpected RCL tarball layout: ${_rcl_topology} is missing.")
endif()
install(
  DIRECTORY "${_rcl_topology}/"
  DESTINATION share/${PROJECT_NAME}/network_topology
)
# Ship the EULA next to the package manifest. Globbed rather than named so a
# version bump does not silently stop installing it; FILES_MATCHING is avoided
# because it mirrors the SDK's whole directory tree into share/.
file(GLOB _rcl_eula LIST_DIRECTORIES false "${kinova_rcl_SOURCE_DIR}/*EULA*.pdf")
if(NOT _rcl_eula)
  message(FATAL_ERROR "No EULA found in the RCL tarball; refusing to ship the SDK without it.")
endif()
install(
  FILES ${_rcl_eula}
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS cmake/kinova_rcl_vendor-extras.cmake)
