cmake_minimum_required(VERSION 3.5)
project(toy_cleaner)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/position_control.py
  scripts/velocity_control.py
  scripts/jetbot_control.py
  scripts/jetbot_control_PID.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
