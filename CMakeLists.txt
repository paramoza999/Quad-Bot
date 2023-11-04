cmake_minimum_required(VERSION 3.5)
project(p5)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


install(DIRECTORY
  config
  description
  launch
  meshes
  worlds
  src
  DESTINATION share/${PROJECT_NAME}
)


install(
  PROGRAMS
    src/rl.py
    
  DESTINATION lib/${PROJECT_NAME}
)


install(
  PROGRAMS
    src/dyn_pos.py
    
  DESTINATION lib/${PROJECT_NAME}


)

install(
  PROGRAMS
    src/test.py
    
  DESTINATION lib/${PROJECT_NAME}


)



ament_package()
