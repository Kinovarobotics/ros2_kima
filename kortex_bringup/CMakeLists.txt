cmake_minimum_required(VERSION 3.5)
project(kortex_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/test_huge_joint_space_jump.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Mock-hardware integration test. It gets its own ROS domain, limited to this
  # host, so it cannot interfere with other tests or with a real arm launch.
  add_launch_test(test/test_kima_mock_bringup.py
    TIMEOUT 400
    ENV ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()
