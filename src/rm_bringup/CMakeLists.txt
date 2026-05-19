cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config PCD rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install helper python scripts as executables. The script lives in the
# workspace-level tools/ folder; we install it under lib/<pkg> so it can be
# referenced via Node(package='rm_bringup', executable='pcd_publisher.py').
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/pcd_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
