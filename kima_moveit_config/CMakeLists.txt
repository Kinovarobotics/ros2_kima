cmake_minimum_required(VERSION 3.5)
project(kima_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
