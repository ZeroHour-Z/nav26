cmake_minimum_required(VERSION 3.8)
project(pointcloud_saver)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# 清理 rosidl 生成的 Python 包目录残留，避免重复 build 时符号链接创建失败
execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory
    "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/pointcloud_saver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
