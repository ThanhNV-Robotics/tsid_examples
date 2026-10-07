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
# MuJoCo release tarballs ship no CMake config; fall back to /opt/mujoco
find_package(mujoco CONFIG QUIET)
if(NOT TARGET mujoco::mujoco)
  find_library(MUJOCO_LIB mujoco PATHS /opt/mujoco/lib REQUIRED NO_DEFAULT_PATH)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION ${MUJOCO_LIB}
    INTERFACE_INCLUDE_DIRECTORIES /opt/mujoco/include)
endif()
find_package(pinocchio REQUIRED)
find_package(tsid QUIET)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED IMPORTED_TARGET urdfdom)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(yaml-cpp QUIET)
pkg_check_modules(tinyxml2 REQUIRED IMPORTED_TARGET tinyxml2)
if(TARGET PkgConfig::tinyxml2 AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 ALIAS PkgConfig::tinyxml2)
endif()

# Preload system tinyxml2 before MuJoCo's internal bundled tinyxml2 to avoid crash in urdfdom
string(PREPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--no-as-needed -ltinyxml2 ")

# -----------------------------------------------------------------------------
# Core Library: bundles sim_interface + Pinocchio + MuJoCo + TSID + GLFW + Eigen
# -----------------------------------------------------------------------------
# Every .cpp in these folders is compiled automatically. CONFIGURE_DEPENDS makes
# the build re-scan the folders, so new files are picked up without re-running
# cmake by hand.
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/sim_interface/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp
)

# Backup copies such as "foo copy.cpp" are never compiled (they would define
# the same symbols twice)
list(FILTER CORE_SOURCES EXCLUDE REGEX " copy\\.cpp$")

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

# One executable per demo. Comment out a block to skip building that demo.

add_executable(biped_robot_leg demos/biped_robot_leg.cpp)
target_link_libraries(biped_robot_leg PRIVATE core)

add_executable(biped_standing demos/biped_standing.cpp)
target_link_libraries(biped_standing PRIVATE core)

add_executable(biped_walking demos/biped_walking.cpp)
target_link_libraries(biped_walking PRIVATE core)

add_executable(init_mujoco_glfw_ui demos/init_mujoco_glfw_ui.cpp)
target_link_libraries(init_mujoco_glfw_ui PRIVATE core)

add_executable(ur10_posturer_task demos/ur10_posturer_task.cpp)
target_link_libraries(ur10_posturer_task PRIVATE core)

add_executable(ur10e_tsid_control demos/ur10e_tsid_control.cpp)
target_link_libraries(ur10e_tsid_control PRIVATE core)
