cmake_minimum_required(VERSION 3.16)
project(LunarRoverAutonomousNavigation C CXX)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/constants)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/deconf)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/nav)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/proavo)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/queue)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/requests)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/route)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/send)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/battery)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/io)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/robot)

if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
endif()

find_package(ompl CONFIG REQUIRED)

find_package(nanoflann CONFIG REQUIRED)

# for ros2 (and gridmap generation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Source files
set(SOURCES
    src/main.c
    
    # Constants
    src/constants/constants.c
    src/constants/config.c
    
    # Core
    src/initialization.c
    src/dead_rover.c
    src/dispatcher.c
    
    # Queue management
    src/queue_management.c
    
    # Objectives system
    src/objectives/objectives_loader.c
    src/objectives/objectives_logic.c
    src/objectives/objective_kdtree.cpp
    # Power station KD-tree
    src/objectives/power_station_kdtree.cpp
    
    # Request handling
    src/requests/requests_process.c
    src/requests/request_handler.c
    
    # Route planning
    src/route/route_planner.c
    src/route/find_power_route.c
    src/route/route_provider.c
    
    # Navigation
    src/nav/nav_main.c
    src/nav/nav_reservationgraph.c
    src/nav/nav_generateinstructions_global.c
    
    # Pathfinding
    src/path_storage.c
    
    # Deconfliction
    src/deconf/deconf_main.c
    src/deconf/deconf_algo.c
    
    # Send system
    src/send/send_main.c
    
    # Proavo pathfinding
    src/proavo/proavo_algo.c
    src/proavo/proavo_cost.c
    src/proavo/proavo_createpath.c
    src/proavo/proavo_pqueue.cpp
    src/proavo/proavo_reedsshepp.cpp
    
    # Priority queue implementations
    src/queue/deconf_pqueue.cpp
    
    # IO operations
    # src/io/construct_full_instructions_generic.c  # DISABLED - switched to pose-based instructions
    # src/io/full_instructions_platform_specific.c  # File doesn't exist
    src/io/platform_specific_instructions_converter.cpp
    src/io/ros2_service_wrapper.cpp
    src/io/rover_action_client.cpp
    src/io/io_send_instructions.cpp
    src/io/robot_initialization.cpp
    
    # Rover config
    src/rover_config.c
    
    # Gridmap
    src/gridmap/gridmap.cpp
    
    # Battery math
    src/battery/batterymath.c
    
    # Timer (platform-specific)
    src/timer_linux.c  # Platform-specific, handled separately
    # src/timer_windows.c  # Platform-specific, handled separately
)

# Header files
set(HEADERS
    # Core headers
    include/structs.h
    include/constants/constants.h
    include/constants/config.h
    include/constants/arrsize_global.h
    include/constants/status_codes.h
    
    # System headers
    include/timer.h
    include/dead_rover.h
    include/queue_management.h
    include/platform.h
    
    # Objectives headers
    include/objectives/objectives_loader.h
    include/objectives/objectives_logic.h
    include/objectives/power_station_kdtree.h
    include/objectives/objective_kdtree.h
    
    # Request headers
    include/requests/requests_process.h
    include/requests/request_handler.h
    
    # Route headers
    include/route/route_provider.h
    include/route/route_planner.h
    include/route/find_power_route.h
    
    # Navigation headers
    include/nav/nav_main.h
    include/nav/generate_instructions_global.h
    include/nav/nav_reservationgraph.h
    
    # Pathfinding headers
    include/path_storage.h
    
    # Deconfliction headers
    include/deconf/deconf_main.h
    include/deconf/deconf_algo.h
    
    # Send headers
    include/send/send_main.h
    
    # Rover configuration
    include/rover_config.h
    include/rover_action_client.h
    
    # Gridmap
    include/gridmap/gridmap.h
    
    # Proavo headers
    include/proavo/proavo_algo.h
    include/proavo/proavo_createpath.h
    include/proavo/proavo_pool.h
    include/proavo/proavo_reedsshepp.h
    include/proavo/proavo_cost.h
    
    # Priority queue headers
    include/deconf_pqueue.h
    include/proavo_pqueue.h
    
    # IO headers
    include/io/platform_specific_instructions_converter.h
    include/io/ros2_service_wrapper.h
    include/io/io_send_instructions.h
    
    # Battery headers
    include/battery/batterymath.h
    
    # Robot headers
    include/robot/robot_initialization.h
)


add_executable(${PROJECT_NAME} ${SOURCES})

# rover shim executable (C++)
add_executable(rover_shim
  shim/rover_shim.cpp
)

ament_target_dependencies(rover_shim
  rclcpp
  rclcpp_action
  robot_interfaces
  nav_msgs
  std_msgs
)

target_include_directories(rover_shim PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


ament_target_dependencies(${PROJECT_NAME}
	rclcpp
	rclcpp_action
	robot_interfaces
    grid_map_ros
    grid_map_sdf
)




target_link_libraries(${PROJECT_NAME}
    m
    ompl::ompl
    nanoflann::nanoflann
    pthread
    ${OpenCV_LIBS}
    grid_map_core::grid_map_core
)


# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    #-Wextra
    -O2
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants
    ${CMAKE_CURRENT_SOURCE_DIR}/include/deconf
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nav
    ${CMAKE_CURRENT_SOURCE_DIR}/include/proavo
    ${CMAKE_CURRENT_SOURCE_DIR}/include/queue
    ${CMAKE_CURRENT_SOURCE_DIR}/include/requests
    ${CMAKE_CURRENT_SOURCE_DIR}/include/route
    ${CMAKE_CURRENT_SOURCE_DIR}/include/send
    ${CMAKE_CURRENT_SOURCE_DIR}/include/battery
    ${CMAKE_CURRENT_SOURCE_DIR}/include/io
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot
    ${CMAKE_CURRENT_SOURCE_DIR}/include/objectives
    ${NANOFLANN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    Eigen3::Eigen
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  rover_shim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  flat_world.png
  rovers.txt
  example_objectives.txt
  DESTINATION lib/${PROJECT_NAME}
)
