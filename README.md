# TSID Examples with MuJoCo

A C++ repository demonstrating Task Space Inverse Dynamics (TSID) controllers integrated with MuJoCo simulation and Pinocchio for robotic manipulators (UR10e, Franka) and bipedal robots.

## Project Structure

- **include/**, **src/**: core library, headers and sources split into the same modules
  (included as `"<module>/<file>.h"`):
  - `common/`: shared data types, CSV logger, kinematics / trajectory helpers.
  - `sim/`: MuJoCo GLFW simulator, viewer and real-time plots.
  - `control/`: YAML → TSID tasks and contacts.
  - `planning/`: walk planner, swing-foot planner, joystick interpreter.
  - `estimation/`: state estimator, contact estimation (momentum observer).
- **config/**: YAML configuration files (joint configurations, gains).
- **demos/**: Example C++ executables and test applications:
  - `ur10_posturer_task.cpp`: UR10e joint posture tracking.
  - `ur10e_tsid_control.cpp`: Operational space control with TSID.
  - `biped_robot_leg.cpp`: Biped leg test.
  - `biped_standing.cpp`: Biped balance and standing task.
  - `biped_walking.cpp`: Biped walking with the walk planner.
  - `init_mujoco_glfw_ui.cpp`: MuJoCo GLFW viewer setup.
- **models/**: Robot descriptions and assets (URDF, MJCF, mesh files).
- **scripts/**: Python analysis and plotting scripts.
- **doc/**: design notes and plans.
- **record/**: CSV logs and plots written by the demos.

Demos find `models/`, `config/` and `record/` through `PROJECT_ROOT_DIR` (set by
CMake), so they can be started from any directory.

## Dependencies

- C++17 compiler
- CMake (>= 3.16)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [TSID](https://github.com/stack-of-tasks/tsid)
- [Eigen3](https://eigen.tuxfamily.org)
- GLFW3, OpenGL, Threads
- urdfdom, yaml-cpp, Boost

## Building

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

## Running Demos

After building, executables are available in the `build/` directory:

```bash
# Run UR10e TSID posture task demo
./build/ur10_posturer_task

# Run Biped Standing demo
./build/biped_standing
```