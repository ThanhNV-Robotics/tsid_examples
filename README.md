# TSID Examples with MuJoCo

A C++ repository demonstrating Task Space Inverse Dynamics (TSID) controllers integrated with MuJoCo simulation and Pinocchio for robotic manipulators (UR10e, Franka) and bipedal robots.

## Project Structure

- **algorithms/**: Custom gait schedulers and control algorithms.
- **config/**: YAML configuration files (joint configurations, gains).
- **demos/**: Example C++ executables and test applications:
  - `ur10_posturer_task.cpp`: UR10e joint posture tracking.
  - `ur10e_tsid_control.cpp`: Operational space control with TSID.
  - `biped_robot_leg.cpp`: Biped leg test.
  - `biped_standing.cpp`: Biped balance and standing task.
  - `init_mujoco_glfw_ui.cpp`: MuJoCo GLFW viewer setup.
- **models/**: Robot descriptions and assets (URDF, MJCF, mesh files).
- **scripts/**: Python analysis and plotting scripts.
- **sim_interface/**: MuJoCo GLFW, simulator and visualization wrapper.
- **utils/**: Helper functions for MuJoCo and kinematics.

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