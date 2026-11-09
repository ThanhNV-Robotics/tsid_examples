# TSID Examples with MuJoCo

A C++ repository demonstrating Task Space Inverse Dynamics (TSID) controllers integrated with MuJoCo simulation and Pinocchio for robotic manipulators (UR10e, Franka) and bipedal robots.

## Project Structure

- **include/**, **src/**: core library, headers and sources split into the same modules
  (included as `"<module>/<file>.h"`):
  - `common/`: shared data types, CSV logger, kinematics / trajectory helpers.
  - `sim/`: MuJoCo GLFW simulator, viewer and real-time plots.
  - `control/`: whole-body controller (`wbc_controller`, `qp_tsid`, `controller_params`).
  - `planning/`: LIP walk planner (`walk_planner`, `lip_task_planner`, see
    [doc/lip_planner.md](doc/lip_planner.md)).
  - `estimation/`: state estimator (IMU + leg kinematics), contact fusion (momentum observer),
    ground-truth estimator.
- **config/**: YAML configuration files (joint configurations, gains).
- **demos/**: Example C++ executables and test applications:
  - `ur10_posturer_task.cpp`: UR10e joint posture tracking.
  - `ur10e_tsid_control.cpp`: Operational space control with TSID.
  - `biped_robot_leg.cpp`: Biped leg test.
  - `biped_walking.cpp`: Biped walking with the whole-body controller, a given number of steps.
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

# Run Biped walking demo
./build/biped_walking
```

### Biped walking

`biped_walking` runs the whole-body controller ported from the ROS 2 project (`biped_wbc_core`,
`wbc_controller`): it lowers to the walking pose, walks the given number of full steps, adds one
closing step that brings the feet side by side, stops, stands for 2 s and closes.

```bash
./build/biped_walking        # num_steps of config/controllers.yaml
./build/biped_walking 8      # 8 full steps, then the closing step
./build/biped_walking 0      # walk until the window is closed
TSID_HEADLESS=1 ./build/biped_walking 4   # hidden windows
```

The camera (`camera_distance`, `camera_azimuth`, `camera_elevation`, `camera_track`) and the video
(`record_video`, `video_fps`) are set in `walk_demo` of `config/controllers.yaml`. The video goes to
`record/biped_walking.mp4`, overwritten at every run, and plays at real speed. Key `T` turns camera tracking on and off during the run.

| File | Contents |
|---|---|
| `config/controllers.yaml` | same keys as the ROS 2 file: `use_time_base`, `torque_mode`, `walk_vx`, `max_yaw_rate`, `use_ground_truth`, `touchdown_confirm_time`, `late_touchdown_timeout`; section `walk_demo`: `num_steps`, `turn_command`, `settle_time` |
| `config/tsid_wbc.yaml` | TSID tasks, contacts, LIP planner (the ROS 2 file) |
| `config/joint_gains.yaml` | drive gains, used with `torque_mode: false` |

The scene `scene_wbc_12dof_v2.xml` is the ROS 2 one, with a 1.5 cm step from x = 0.6 m. At
`walk_vx: 0.1` a walk of 8 steps reaches it: time-based walking falls there, event-based walking
(`use_time_base: false`) walks over it. Control runs at 500 Hz with 2 physics steps per cycle, as
`lockstep:=true` in the ROS 2 simulation.
