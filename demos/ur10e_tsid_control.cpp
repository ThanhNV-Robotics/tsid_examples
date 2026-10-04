// ============================================================================
// Example: Task Space Inverse Dynamics (TSID) Control for UR10e in MuJoCo
//
// This example demonstrates:
//   1. Loading the UR10e robot into TSID (using Pinocchio + eiquadprog-fast).
//   2. Loading the UR10e scene into MuJoCo with Robot_Simulator visualization.
//   3. Setting up an Operational Space task (TaskSE3Equality) for the
//   end-effector
//      to track a smooth 3D circular trajectory in Cartesian space.
//   4. Setting up a secondary Joint Posture task (TaskJointPosture) in the
//      null space of the end-effector task.
//   5. Solving the Hierarchical Quadratic Program (HQP) at each control step
//      to compute inverse dynamics torques (including gravity compensation).
//   6. Measuring and displaying the QP solver execution time in microseconds.
//   7. Applying the computed torques to MuJoCo actuators.
//   8. Visualizing the robot, desired target, and interactive perturbations in
//      the unified Robot_Simulator window.
//
// Interactive Controls in Window:
//   - Space / 1: Pause / Resume simulation
//   - Double-click body + Ctrl + Left-drag: Apply twist perturbation
//   - Double-click body + Ctrl + Right-drag: Apply force perturbation
//   - Right-click + drag: Rotate camera; Scroll: Zoom
// ============================================================================

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

// TSID headers
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

// Unified Simulator
#include "Robot_Simulator.h"

using namespace tsid;
using namespace tsid::robots;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::trajectories;
using namespace std;

int main(int argc, char **argv) {
  const std::string xml_path = "models/ur10e/scene_ur10.xml";
  const std::string urdf_path = "models/ur10e/ur10e.urdf";

  std::printf("====================================================\n");
  std::printf("  UR10e TSID Operational Space Control with MuJoCo  \n");
  std::printf("====================================================\n");

  // ------------------------------------------------------------------------
  // 1. Initialize MuJoCo model & data
  // ------------------------------------------------------------------------
  char loadError[1024] = "";
  mjModel *m =
      mj_loadXML(xml_path.c_str(), nullptr, loadError, sizeof(loadError));
  if (!m) {
    std::fprintf(stderr, "Failed to load MuJoCo scene '%s': %s\n",
                 xml_path.c_str(), loadError);
    return 1;
  }
  mjData *d = mj_makeData(m);

  // Initial joint configuration (home pose)
  Eigen::VectorXd q_init(6);
  q_init << -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0;
  for (int i = 0; i < 6; ++i) {
    d->qpos[i] = q_init(i);
    d->qvel[i] = 0.0;
  }
  mj_forward(m, d);

  // ------------------------------------------------------------------------
  // 2. Initialize TSID Robot Wrapper & Formulation
  // ------------------------------------------------------------------------
  std::vector<std::string> package_dirs;
  RobotWrapper robot(urdf_path, package_dirs, false);
  std::printf("[TSID] Loaded URDF '%s'\n", urdf_path.c_str());
  std::printf("[TSID] Robot DOF: nq=%d, nv=%d, na=%d\n", robot.nq(), robot.nv(),
              robot.na());

  InverseDynamicsFormulationAccForce tsid("tsid-ur10e", robot);

  // Current state vectors
  Eigen::VectorXd q = q_init;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.nv());

  // ------------------------------------------------------------------------
  // 3. Define Tasks: End-Effector Tracking & Posture Regularization
  // ------------------------------------------------------------------------
  const std::string ee_frame_name = "wrist_3_link";
  auto eeTask =
      std::make_shared<TaskSE3Equality>("task-ee", robot, ee_frame_name);
  eeTask->Kp(150.0 * Eigen::VectorXd::Ones(6));
  eeTask->Kd(2.0 * std::sqrt(150.0) * Eigen::VectorXd::Ones(6));

  // Compute initial end-effector position from initial forward kinematics
  tsid.computeProblemData(0.0, q, v);
  auto ee_frame_id = robot.model().getFrameId(ee_frame_name);
  pinocchio::SE3 oMf_init = robot.framePosition(tsid.data(), ee_frame_id);
  const Eigen::Vector3d p_center = oMf_init.translation();
  const Eigen::Matrix3d R_ref = oMf_init.rotation();

  std::printf("[TSID] Initial End-Effector Position: [%.3f, %.3f, %.3f]\n",
              p_center.x(), p_center.y(), p_center.z());

  // Set initial reference for EE task (weight = 1.0, priority = 1)
  eeTask->setReference(oMf_init);
  tsid.addMotionTask(*eeTask, 1.0, 1);

  // Secondary Postural Task (null-space regularization, weight = 1e-3, priority
  // = 1)
  auto postureTask = std::make_shared<TaskJointPosture>("task-posture", robot);
  postureTask->Kp(30.0 * Eigen::VectorXd::Ones(robot.nv()));
  postureTask->Kd(2.0 * std::sqrt(30.0) * Eigen::VectorXd::Ones(robot.nv()));
  TrajectorySample postureSample(robot.nv());
  postureSample.setValue(q_init);
  postureTask->setReference(postureSample);
  tsid.addMotionTask(*postureTask, 1e-3, 1);

  // ------------------------------------------------------------------------
  // 4. Initialize HQP Solver (eiquadprog-fast)
  // ------------------------------------------------------------------------
  auto solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                  "eiquadprog-fast");
  solver->resize(tsid.nVar(), tsid.nEq(), tsid.nIn());

  // ------------------------------------------------------------------------
  // 5. Initialize Robot_Simulator
  // ------------------------------------------------------------------------
  Robot_Simulator sim(m, d);
  sim.iniGLFW();
  sim.createWindow("UR10e TSID Operational Space Control", /*saveVideo=*/false);

  std::printf("[SIM] Window initialized. Starting simulation loop...\n");
  std::printf(
      "[SIM] Press '1' to pause/resume; Drag robot body to test compliance.\n");

  // Trajectory parameters for end-effector circular motion
  const double radius = 0.12;    // meters
  const double frequency = 0.25; // Hz (1 circle every 4 seconds)
  const double omega = 2.0 * M_PI * frequency;

  const float targetColor[4] = {0.1f, 0.9f, 0.2f, 0.9f}; // vibrant green

  // Benchmarking counters for QP solve time
  uint64_t step_count = 0;
  double total_solve_time_us = 0.0;
  double min_solve_time_us = 1e9;
  double max_solve_time_us = 0.0;

  // ------------------------------------------------------------------------
  // 6. Simulation & Control Loop
  // ------------------------------------------------------------------------
  while (!sim.shouldClose()) {
    double simstart = d->time;

    // Step physics at ~60 Hz visual frame rate
    while (d->time - simstart < 1.0 / 60.0 && sim.runSim) {
      double t = d->time;

      // 6.1 Read state from MuJoCo
      for (int i = 0; i < 6; ++i) {
        q(i) = d->qpos[i];
        v(i) = d->qvel[i];
      }

      // 6.2 Compute desired end-effector trajectory (circle in X-Z plane)
      Eigen::Vector3d p_des = p_center;
      p_des.x() += radius * std::sin(omega * t);
      p_des.z() += radius * (1.0 - std::cos(omega * t));

      pinocchio::SE3 M_des(R_ref, p_des);
      eeTask->setReference(M_des);

      // 6.3 Formulate problem data
      const HQPData &hqpData = tsid.computeProblemData(t, q, v);

      // 6.4 Benchmark and solve the QP problem
      auto t_start = std::chrono::high_resolution_clock::now();
      const HQPOutput &sol = solver->solve(hqpData);
      auto t_end = std::chrono::high_resolution_clock::now();

      double solve_time_us =
          std::chrono::duration<double, std::micro>(t_end - t_start).count();
      step_count++;
      total_solve_time_us += solve_time_us;
      min_solve_time_us = std::min(min_solve_time_us, solve_time_us);
      max_solve_time_us = std::max(max_solve_time_us, solve_time_us);

      // Print solve time statistics periodically (every 100 control steps)
      if (step_count % 100 == 0) {
        std::printf("[QP Solve Time] current: %6.1f us (%5.3f ms) | avg: %6.1f "
                    "us | min: %6.1f us | max: %6.1f us (samples: %lu)\n",
                    solve_time_us, solve_time_us * 1e-3,
                    total_solve_time_us / step_count, min_solve_time_us,
                    max_solve_time_us, step_count);
        std::fflush(stdout);
      }

      if (sol.status == HQP_STATUS_OPTIMAL) {
        Eigen::VectorXd tau = tsid.getActuatorForces(sol);

        // 6.5 Apply computed torques to MuJoCo actuators
        for (int i = 0; i < 6; ++i) {
          d->ctrl[i] = tau(i);
        }
      } else {
        std::fprintf(stderr, "[TSID] Warning: Solver status %d at t=%.3f\n",
                     sol.status, t);
      }

      // 6.6 Apply interactive user perturbations and step physics
      sim.applyPerturbation();
      mj_step(m, d);
    }

    // 6.7 Visualize desired target and tracking indicator
    {
      double t = d->time;
      Eigen::Vector3d p_des = p_center;
      p_des.x() += radius * std::sin(omega * t);
      p_des.z() += radius * (1.0 - std::cos(omega * t));

      pinocchio::SE3 oMf_current =
          robot.framePosition(tsid.data(), ee_frame_id);
      Eigen::Vector3d p_actual = oMf_current.translation();

      // Line segment between actual end-effector and target
      sim.addLine(p_actual, p_des, targetColor, 3.0);

      // Small 3D arrow at the target position pointing upward
      Eigen::Vector3d arrow_dir(0.0, 0.0, 0.04);
      sim.addArrow(p_des, arrow_dir, 1.0, targetColor, 0.015);
    }

    // 6.8 Render frame
    sim.updateScene();
  }

  // Cleanup
  sim.Close();
  return 0;
}