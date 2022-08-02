cmake_minimum_required(VERSION 3.5)
project(maze_vision)


####################
## Compiler Setup ##
####################

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find Ament Support Packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


#############
## Install ##
#############

# Install Python Modules (must contain an __init__.py file)
ament_python_install_package("maze_vision")
ament_python_install_package("maze_solver")

# Install Misc ROS Elements
install(
  DIRECTORY config launch sim
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS nodes/node_maze_vision.py
  DESTINATION lib/${PROJECT_NAME}
)


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


## ###############################
## Amend Specific Configuration ##
##################################

ament_package()
