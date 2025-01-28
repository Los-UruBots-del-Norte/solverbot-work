cmake_minimum_required(VERSION 3.8)
project(motor_board_md49_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY motor_board_md49
  DESTINATION lib/${PROJECT_NAME}
)

# Instala los archivos Python como ejecutables
install(
  PROGRAMS motor_board_md49/motor_board.py
  PROGRAMS motor_board_md49/ros_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()