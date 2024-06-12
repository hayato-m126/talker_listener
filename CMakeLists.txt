cmake_minimum_required(VERSION 3.14)
project(talker_listener)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/talker_node.py
  scripts/listener_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
