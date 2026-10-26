cmake_minimum_required(VERSION 3.8)
project(roboracer_gazebo)

find_package(ament_cmake REQUIRED)

# Install all directories
install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
