#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
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
#include "fstream"

#include "utils.h" // supporting functions

using namespace tsid;
using namespace tsid::robots;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::trajectories;
using namespace std;
using namespace Eigen;
using namespace utils;

int main(int argc, char **argv) {
  const std::string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
  const std::string URDF_PATH = "models/ur10e/ur10e.urdf";

  std::printf("====================================================\n");
  std::printf("  UR10e TSID Operational Space Control with MuJoCo  \n");
  std::printf("====================================================\n");

  // ------------------------------------------------------------------------
  // 1. Initialize MuJoCo model & data
  // ------------------------------------------------------------------------
  char loadError[1024] = "";
  mjModel *m;
  mjData *d;
  utils::loadMjXml(m, d, XML_PATH);
  utils::printMjModelInfo(m);
  printf("Compiled xml model done");

  Robot_Simulator sim(m, d);
  sim.iniGLFW();
  sim.createWindow("UR10e TSID Operational Space Control", /*saveVideo=*/false);
  cout << "[SIM] Press '1' to pause/resume; Drag robot body to test compliance.\n ";
         //   // Initial joint configuration (home pose)
         //   Eigen::VectorXd q_init(6);
         //   q_init << -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0;
         //   utils::setRobotInitConfiguration(m, d, q_init);
         //   printf("Set robot initial configuration");

         //   //
         //   ------------------------------------------------------------------------
         //   // 2. Initialize TSID Robot Wrapper & Formulation
         //   //
         //   ------------------------------------------------------------------------
         //   std::vector<std::string> package_dirs;
         //   RobotWrapper robot(URDF_PATH, package_dirs, false);
         //   std::printf("[TSID] Loaded URDF '%s'\n", URDF_PATH.c_str());
         //   std::printf("[TSID] Robot DOF: nq=%d, nv=%d, na=%d\n", robot.nq(),
         //   robot.nv(),
         //               robot.na());

         //   InverseDynamicsFormulationAccForce tsid("tsid-ur10e", robot,
         //   true);

         //   //
         //   ------------------------------------------------------------------------
         //   // 3. Define Tasks: End-Effector Tracking & Posture Regularization
         //   //
         //   ------------------------------------------------------------------------
         //   auto posturer_task = std::make_shared<TaskJointPosture>(
         //       "task-posture", robot); // TaskJointPosture is a joint-space
         //       motion task

         //   // define gain Kp and Kd for the task
         //   // for a posture task, we track the joint-space trajectory
         //   // it computes desired joint acceleration
         //   // ddq_cmd = ddq_ref + Kp * (q_ref - q) + Kd * (v_ref - v)
         //   // those gains are corresponding to joint Kp and Kd
         //   posturer_task->Kp(100.0 * Eigen::VectorXd::Ones(robot.na()));
         //   posturer_task->Kd(2.0 * std::sqrt(100.0) *
         //   Eigen::VectorXd::Ones(robot.na()));

         //   std::cout << "Task info: " << posturer_task->name() << endl;

         //   // weight of the cost function
         //   // the cost function in this case is
         //   // min 0.5*W*||ddq - ddq_cmd||^2
         //   // in general we find joint torque tau s.t ddq = ddq_mcd
         //   const double w_posture = 1.0;
         //   tsid.addMotionTask(*posturer_task, w_posture, 1.0,
         //                      0.0); // task, weight, priority, transition
         //                      time
         //   cout << "Config task done" << endl;
         //   //
         //   ------------------------------------------------------------------------
         //   // 4. Initialize HQP Solver (eiquadprog-fast)
         //   //
         //   ------------------------------------------------------------------------
         //   auto solver =
         //   SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
         //                                                   "QP solver");

         //   // Reference sinsoidal trajectory
         //   VectorXd amp(6);
         //   amp << 0.2, 0.3, 0.2, 0, 0, 0;
         //   VectorXd phi(6);
         //   phi << 0, 0.5 * M_PI, 0, 0, 0, 0;
         //   VectorXd freq(6);
         //   freq << 1.0, 0.5, 0.3, 0, 0, 0;
         //   VectorXd two_pi_f = 2.0 * M_PI * freq;
         //   VectorXd two_pi_f_amp = two_pi_f.cwiseProduct(amp);
         //   VectorXd two_pi_f_squared_amp =
         //   two_pi_f.cwiseProduct(two_pi_f_amp);

         //   cout << "Config reference trajectory done" << endl;

         //   TrajectorySample sample(robot.na());
         //   VectorXd q_ref(robot.na());
         //   VectorXd v_ref(robot.na());
         //   VectorXd dv_ref(robot.na());

         //   // State vectors:
         //   VectorXd q(robot.nq());
         //   VectorXd v(robot.nv());

         //   //
         //   ------------------------------------------------------------------------
         // 5. Initialize Robot_Simulator
         //

  //   //
  //   ------------------------------------------------------------------------
  //   // csv file log
  //   //
  //   ------------------------------------------------------------------------
  //   std::ofstream datalog(("record/ur10_joint_space_control.csv"));
  //   vector<string> joint_names = utils::getJointNames(m);
  //   datalog << "time";
  //   for (const auto &name : joint_names)
  //     datalog << "," << name << "_pos_cmd";
  //   for (const auto &name : joint_names)
  //     datalog << "," << name << "_pos_measured";
  //   datalog << "\n";

  //   //
  //   ------------------------------------------------------------------------
  //   // 6. Simulation & Control Loop
  //   //
  //   ------------------------------------------------------------------------
  //   const double simulation_time = 6;
  //   while (!sim.shouldClose() && d->time < simulation_time) {
  //     double simstart = d->time;

  //     // Step physics at ~60 Hz visual frame rate
  //     while (d->time - simstart < 1.0 / 60.0 && sim.runSim) {
  //       double t = d->time;

  //       // 7.1 Read current joint state from MuJoCo
  //       for (int i = 0; i < robot.na(); ++i) {
  //         q(i) = d->qpos[i];
  //         v(i) = d->qvel[i];
  //       }

  //       // 7.2 Compute Sinusoidal Reference Trajectory
  //       for (int i = 0; i < 6; ++i) {
  //         double angle = two_pi_f(i) * t + phi(i);
  //         q_ref(i) = q_init(i) + amp(i) * std::sin(angle);
  //         v_ref(i) = two_pi_f_amp(i) * std::cos(angle);
  //         dv_ref(i) = -two_pi_f_squared_amp(i) * std::sin(angle);
  //       }

  //       sample.setValue(q_ref);
  //       sample.setDerivative(v_ref);
  //       sample.setSecondDerivative(dv_ref);
  //       posturer_task->setReference(sample);

  //       // 7.3 Solve HQP Problem
  //       const auto &hqpData = tsid.computeProblemData(t, q, v);
  //       const auto &sol = solver->solve(hqpData);

  //       // 7.4 Extract torques & apply to MuJoCo actuators
  //       Eigen::VectorXd tau = tsid.getActuatorForces(sol);
  //       for (int i = 0; i < robot.na(); ++i) {
  //         d->ctrl[i] = tau(i); // set torque to mujoco simu
  //       }

  //       // log data
  //       datalog << t;
  //       // 1. Commanded positions (q_ref)
  //       for (int i = 0; i < robot.na(); ++i) {
  //         datalog << "," << q_ref(i);
  //       }
  //       // 2. Measured positions from MuJoCo (d->qpos)
  //       for (int i = 0; i < robot.na(); ++i) {
  //         datalog << "," << d->qpos[i];
  //       }
  //       datalog << "\n";

  //       // 6.6 Apply interactive user perturbations and step physics
  //       sim.applyPerturbation();
  //       mj_step(m, d);
  //     }

  //     // 6.8 Render frame
  //     sim.updateScene();
  //   }

  //   // Flush and close log file
  //   datalog.close();
  //   std::cout << "Data saved to record/ur10_joint_space_control.csv" <<
  //   std::endl;
  //   // Cleanup
  //   sim.Close();
  return 0;
}