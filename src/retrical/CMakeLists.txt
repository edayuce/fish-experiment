# ===================================================================
# Compiling for BASLER Camera
# ===================================================================
cmake_minimum_required(VERSION 3.0.2)
project(rectrial)

set(CMAKE_CXX_STANDARD 11)
add_definitions("-std=gnu++11")

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_runtime
  sensor_msgs
  image_transport
  cv_bridge
  roslint
  message_generation
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Pylon for Basler Camera
# This assumes the Pylon SDK is installed in a standard location.
find_package(Pylon REQUIRED)


# ===================================================================
# ROS Messages
# ===================================================================
add_message_files(
  FILES
  pub_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


# ===================================================================
# Catkin Package Configuration
# ===================================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs image_transport cv_bridge
)


# ===================================================================
# Include Directories
# ===================================================================
# Add directories needed for compilation
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pylon_INCLUDE_DIRS}
)


# ===================================================================
# Build Libraries & Executables
# ===================================================================
# NOTE: All C++ source files (.cpp) should be in a 'src' folder,
# not a 'scripts' folder. The paths below have been corrected.

# --- Libraries ---
add_library(GlWindow STATIC
  scripts/GLWindow.cpp
  scripts/gl/src/glad.c
)
# Add include directory for this specific library
target_include_directories(GlWindow PUBLIC scripts/gl/include)


# --- Executables ---
add_executable(genicam_cpp_demo      scripts/camera_node.cpp)
add_executable(experiment_controller scripts/experiment_controller.cpp)
add_executable(rec_node              scripts/rec_node.cpp)
add_executable(HelloEposCmd          scripts/HelloEposCmd.cpp)
add_executable(real_time_tracker     scripts/real_time_tracker.cpp)
add_executable(main_node             scripts/main_node.cpp)



# ===================================================================
# Link Libraries
# ===================================================================
# Link libraries to their corresponding executables

# Link Basler camera node
target_link_libraries(genicam_cpp_demo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Pylon_LIBRARIES}
)

# Link other nodes
target_link_libraries(rec_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(experiment_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  GlWindow
  # You may need to add -lJetsonGPIO here if you are on a Jetson
)

target_link_libraries(HelloEposCmd
  ${catkin_LIBRARIES}
  # You need to ensure the EposCmd library is found and linked, for example:
  # -lEposCmd
)



# ===================================================================
# Dependencies
# ===================================================================
# Ensure messages are generated before nodes that use them are built
add_dependencies(genicam_cpp_demo      ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(experiment_controller ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rec_node              ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(HelloEposCmd          ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(main_node             ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(real_time_tracker     ${PROJECT_NAME}_generate_messages_cpp)
