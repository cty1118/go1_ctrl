cmake_minimum_required(VERSION 3.10)
project(go1_ctrl)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs unitree_legged_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs unitree_legged_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# ---- Unitree legged SDK (only needed for the real robot) ----
# Build with:  catkin_make -DUNITREE_SDK_PATH=$HOME/unitree_legged_sdk
set(UNITREE_SDK_PATH "" CACHE PATH "Path to unitree_legged_sdk (v3.8.x). Empty = simulation only.")
set(GO1_WITH_SDK OFF)
if(UNITREE_SDK_PATH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(_sdk_arch arm64)
  else()
    set(_sdk_arch amd64)
  endif()
  find_library(UNITREE_SDK_LIB unitree_legged_sdk
    HINTS ${UNITREE_SDK_PATH}/lib/cpp/${_sdk_arch} ${UNITREE_SDK_PATH}/lib/${_sdk_arch}
          ${UNITREE_SDK_PATH}/lib ${UNITREE_SDK_PATH}/build)
  if(UNITREE_SDK_LIB)
    set(GO1_WITH_SDK ON)
    message(STATUS "go1_ctrl: real robot support ON (${UNITREE_SDK_LIB})")
  else()
    message(WARNING "go1_ctrl: libunitree_legged_sdk not found under ${UNITREE_SDK_PATH}")
  endif()
else()
  message(STATUS "go1_ctrl: UNITREE_SDK_PATH not set, building for simulation only")
endif()

# ---- controller modules (one owner each, see the owner note at the top of each file) ----
add_library(go1_ctrl_core
  src/fsm.cpp                 # Member 1
  src/sim_io.cpp              # Member 1
  src/real_io.cpp             # Member 1
  src/kinematics.cpp          # Member 2
  src/estimator.cpp           # Member 2
  src/gait_scheduler.cpp      # Member 3
  src/swing_controller.cpp    # Member 4
  src/foot_planner.cpp        # Member 4
  src/stance_controller.cpp   # Member 5
  src/contact_detector.cpp    # Member 6
  src/transition_blender.cpp) # Member 6
add_dependencies(go1_ctrl_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(go1_ctrl_core ${catkin_LIBRARIES})
if(GO1_WITH_SDK)
  target_compile_definitions(go1_ctrl_core PRIVATE GO1_WITH_SDK)
  target_include_directories(go1_ctrl_core PRIVATE ${UNITREE_SDK_PATH}/include)
  target_link_libraries(go1_ctrl_core ${UNITREE_SDK_LIB} pthread)
endif()

add_executable(go1_ctrl_node src/main.cpp)
target_link_libraries(go1_ctrl_node go1_ctrl_core ${catkin_LIBRARIES})

add_executable(keyboard src/keyboard.cpp)
add_dependencies(keyboard ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard ${catkin_LIBRARIES})

# Member 2: kinematics tests
if(CATKIN_ENABLE_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_kinematics.cpp)
  catkin_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics go1_ctrl_core ${catkin_LIBRARIES})
endif()
