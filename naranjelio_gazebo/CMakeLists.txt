cmake_minimum_required(VERSION 3.8)
project(naranjelio_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/bridge_gz.sh
  scripts/run_state_publisher.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()