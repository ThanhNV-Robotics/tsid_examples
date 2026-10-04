cmake_minimum_required(VERSION 3.16)
project(tsid_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DCOAL_DISABLE_HPP_FCL_WARNINGS)

# Symlink compile_commands.json to source root and parent workspace root for Antigravity IDE IntelliSense
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_CURRENT_SOURCE_DIR}/../compile_commands.json)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Output binaries directly into build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Ignore Conda prefix to prevent ABI conflicts with system libraries
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

# Set prefix paths for system-wide, robotpkg, and local packages
list(APPEND CMAKE_PREFIX_PATH
  "/opt/openrobots"
  "/opt/mujoco"
  "/usr/local"
)

# -----------------------------------------------------------------------------
# System-wide packages
# -----------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(mujoco REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tsid QUIET)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED IMPORTED_TARGET urdfdom)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(yaml-cpp QUIET)
find_package(tinyxml2 CONFIG REQUIRED)

# Preload system tinyxml2 before MuJoCo's internal bundled tinyxml2 to avoid crash in urdfdom
string(PREPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--no-as-needed -ltinyxml2 ")

# -----------------------------------------------------------------------------
# Core Library: bundles sim_interface + Pinocchio + MuJoCo + TSID + GLFW + Eigen
# -----------------------------------------------------------------------------
set(CORE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/sim_interface/Robot_Simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/cheat_state_estimator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/data_logger.cpp
)

add_library(core STATIC ${CORE_SOURCES})
target_include_directories(core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/sim_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithms
  ${CMAKE_CURRENT_SOURCE_DIR}/utils
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  /usr/local/include
)
target_compile_definitions(core PUBLIC
  COAL_DISABLE_HPP_FCL_WARNINGS
)
target_link_libraries(core PUBLIC
  tinyxml2::tinyxml2
  pinocchio::pinocchio
  PkgConfig::urdfdom
  mujoco::mujoco
  glfw
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
  OpenGL::GL
  Threads::Threads
  ${CMAKE_DL_LIBS}
)

if(TARGET tsid::tsid)
  target_link_libraries(core PUBLIC tsid::tsid)
endif()

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(core PUBLIC yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(core PUBLIC yaml-cpp)
endif()

# Alias sim_interface to core for backward compatibility
add_library(sim_interface ALIAS core)

# -----------------------------------------------------------------------------
# Executables (demos, tests, etc.)
# -----------------------------------------------------------------------------

add_executable(init_mujoco_glfw_ui demos/init_mujoco_glfw_ui.cpp)
target_link_libraries(init_mujoco_glfw_ui PRIVATE core)

add_executable(ur10e_tsid_control demos/ur10e_tsid_control.cpp)
target_link_libraries(ur10e_tsid_control PRIVATE core)

add_executable(biped_robot_leg demos/biped_robot_leg.cpp)
target_link_libraries(biped_robot_leg PRIVATE core)

add_executable(ur10_posturer_task demos/ur10_posturer_task.cpp)
target_link_libraries(ur10_posturer_task PRIVATE core)

add_executable(biped_standing demos/biped_standing.cpp)
target_link_libraries(biped_standing PRIVATE core)
