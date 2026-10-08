cmake_minimum_required(VERSION 3.5)
project(kima_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Mock-hardware integration test. It gets its own ROS domain, limited to this
  # host, so it cannot interfere with other tests or with a real arm launch.
  add_launch_test(test/test_moveit_mock.py
    TIMEOUT 400
    ENV ROS_DOMAIN_ID=62 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()
