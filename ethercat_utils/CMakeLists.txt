cmake_minimum_required(VERSION 3.8)
project(ethercat_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ethercat_utils_msgs REQUIRED)
find_package(controller_interface REQUIRED)

# add all external dependencies to a list
list(APPEND DEPENDENCIES  "rclcpp"  
                          "pluginlib"
                          "std_msgs"
                          "std_srvs"
                          "control_msgs"
                          "controller_manager_msgs"
                          "builtin_interfaces"
                          "ethercat_utils_msgs"
                          "controller_interface")


# add include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###########
## Build ##
###########

# add library
add_library(${PROJECT_NAME} 
              SHARED
              src/${PROJECT_NAME}/battery_cell_utils.cpp
              src/${PROJECT_NAME}/cia402_slave_utils.cpp
              src/${PROJECT_NAME}/gpio_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)    
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

pluginlib_export_plugin_description_file(controller_interface gpio_controller_plugins.xml)

# add executable
add_executable(cia402_slave_manager src/cia402_slave_manager.cpp)
ament_target_dependencies(cia402_slave_manager ${DEPENDENCIES})
target_link_libraries(cia402_slave_manager ${PROJECT_NAME})

add_executable(battery_cell_utils_manager src/battery_cell_utils_manager.cpp)
ament_target_dependencies(battery_cell_utils_manager ${DEPENDENCIES})
target_link_libraries(battery_cell_utils_manager ${PROJECT_NAME})

#############
## Install ##
#############

# install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executable
install(TARGETS cia402_slave_manager battery_cell_utils_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

############
## Export ##
############

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
  cia402_slave_utils
  battery_cell_utils
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
