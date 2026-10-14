cmake_minimum_required(VERSION 3.8)
project(picaso_driver)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/picaso_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ../sweeps/ DESTINATION share/${PROJECT_NAME}/sweeps
  FILES_MATCHING PATTERN "*.csv")
ament_package()
