#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

// TSID headers
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

// Unified Simulator
#include "Robot_Simulator.h"
#include "fstream"

#include "cheat_state_estimator.h"
#include "utils.h" // supporting functions
#include "data_logger.h"
#include "data_type.h"

using namespace tsid;
using namespace tsid::robots;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::trajectories;
using namespace std;
using namespace Eigen;

using tsid::contacts::Contact6d;

int main(int argc, char **argv) {
  const string XML_PATH = "models/mjcf/scene_floatingbase_12dof_v2.xml";
  const string URDF_PATH = "models/urdf/v2_biped_robot_12dof.urdf";
  const string JOINT_PD_CF_PATH = "config/12dof_joint_config.yaml";

  std::printf("====================================================\n");
  std::printf("  Biped TSID Standing Posture Control with MuJoCo   \n");
  std::printf("====================================================\n");

  // ------------------------------------------------------------------------
  // 1. Initialize Robot_Simulator with MuJoCo XML scene
  // ------------------------------------------------------------------------
  Robot_Simulator sim(XML_PATH);
  sim.init("Biped TSID Standing Posture Control", /*saveVideo=*/false);
  sim.printModelInfo();
  std::printf("Compiled xml model done\n");

  // ------------------------------------------------------------------------
  // 2. TSID Robot Wrapper
  // ------------------------------------------------------------------------
  std::vector<std::string> package_dirs;
  // The URDF has no root joint (its floating joint is commented out), so the
  // free-flyer must be added here. Without it TSID assumes the torso is bolted
  // to the world and only computes the torques needed to swing the legs in
  // the air -- far too little to carry the body weight through the feet.
  RobotWrapper robot(URDF_PATH, package_dirs, pinocchio::JointModelFreeFlyer(), false);
  std::printf("[TSID] Loaded URDF '%s'\n", URDF_PATH.c_str());
  std::printf("[TSID] Robot DOF: nq=%d, nv=%d, na=%d\n", robot.nq(), robot.nv(),
              robot.na());

  // ------------------------------------------------------------------------
  // 3. Compile Pinocchio model and data for standing IK
  // ------------------------------------------------------------------------
  pinocchio::Model pin_model;
  pinocchio::urdf::buildModel(URDF_PATH, pin_model);
  pinocchio::Data pin_data(pin_model);
  std::printf("[Pinocchio] Model compiled from '%s' (nq=%d, nv=%d)\n",
              URDF_PATH.c_str(), pin_model.nq, pin_model.nv);

  const double hip_width = 0.27;
  const double init_base_height = 0.78;
  VectorXd qa_init = utils::computeIntialStandConfig(pin_model, pin_data, hip_width,
                                                     init_base_height);
  std::cout << "Target Initial Standing Configuration:\n" << qa_init.transpose() << std::endl;

  // Apply initial slightly bent configuration to MuJoCo to avoid kinematic singularity at start
  VectorXd q_bent = pinocchio::neutral(pin_model);
  q_bent[3] = 0.1;    // left knee
  q_bent[9] = 0.1;    // right knee
  q_bent[0] = -0.08;  // left hip pitch
  q_bent[6] = -0.08;  // right hip pitch
  q_bent[5] = -0.05;  // left ankle
  q_bent[11] = -0.08; // right ankle

  // Foot sole geometry (matches the "foot1" collision box in the MJCF), expressed
  // in the ankle-pitch link frame: box center (0.04, 0, -0.034), half-size
  // (0.09, 0.03, 0.006).
  const string LF_FRAME = "left_ankle_pitch_link";
  const string RF_FRAME = "right_ankle_pitch_link";
  const double sole_z = -0.040;
  Matrix3Xd contact_points(3, 4);
  contact_points << -0.05, -0.05, 0.13, 0.13,
                    -0.03,  0.03, -0.03, 0.03,
                    sole_z, sole_z, sole_z, sole_z; // list of contact point on the contact sole
  const Vector3d contact_normal(0.0, 0.0, 1.0); // normal contact force direction w.r.t contact frame

  // // Place the base so that the soles of the bent configuration rest on the floor
  VectorXd q(robot.nq());
  VectorXd v = VectorXd::Zero(robot.nv());
  q = pinocchio::neutral(robot.model());
  q.tail(robot.na()) = q_bent;
  {
    pinocchio::Data d_tmp(robot.model());
    pinocchio::framesForwardKinematics(robot.model(), d_tmp, q);
    const double foot_z = std::min(
        d_tmp.oMf[robot.model().getFrameId(LF_FRAME)].translation().z(),
        d_tmp.oMf[robot.model().getFrameId(RF_FRAME)].translation().z());
    q[2] = -(foot_z + sole_z) + 1e-3;
  }
  VectorXd qpos_mj(sim.nq());
  qpos_mj << q.head<3>(), 1.0, 0.0, 0.0, 0.0, q_bent; // MuJoCo quat (w,x,y,z)
  sim.setInitConfiguration(qpos_mj);
  std::cout << "Applied initial bent configuration to MuJoCo:\n" << qpos_mj.transpose() << std::endl;

  // TSID Formulation init
  InverseDynamicsFormulationAccForce tsid("tsid-biped", robot);
  Cheat_StateEstimator state_estimator(sim.model(), sim.data());
  utils::robotStateToPinocchio(state_estimator.estimate(sim.getRobotSensorValues()), q, v);
  tsid.computeProblemData(0.0, q, v);
  const pinocchio::Data &data = tsid.data();

  // Rigid 6D contacts on both feet: these let TSID use ground reaction forces
  // to hold the body up (and constrain them to friction cones).
  const double mu = 0.8, f_min = 0.0, f_max = 1000.0;
  const double kp_contact = 10.0, w_force_reg = 1e-5;
  auto contact_lf = std::make_shared<Contact6d>("contact_lfoot", robot, LF_FRAME,
                                                contact_points, contact_normal,
                                                mu, f_min, f_max);
  auto contact_rf = std::make_shared<Contact6d>("contact_rfoot", robot, RF_FRAME,
                                                contact_points, contact_normal,
                                                mu, f_min, f_max);
  for (auto &c : {contact_lf, contact_rf}) {
    c->Kp(kp_contact * VectorXd::Ones(6));
    c->Kd(2.0 * std::sqrt(kp_contact) * VectorXd::Ones(6));
  }
  contact_lf->setReference(robot.framePosition(data, robot.model().getFrameId(LF_FRAME)));
  contact_rf->setReference(robot.framePosition(data, robot.model().getFrameId(RF_FRAME)));
  tsid.addRigidContact(*contact_lf, w_force_reg);
  tsid.addRigidContact(*contact_rf, w_force_reg);

  // Keep the CoM horizontally centred over the two soles (height is left to
  // the posture task).
  const double kp_com = 50.0, w_com = 1.0;
  auto com_task = std::make_shared<TaskComEquality>("task-com", robot);
  com_task->Kp(kp_com * VectorXd::Ones(3));
  com_task->Kd(2.0 * std::sqrt(kp_com) * VectorXd::Ones(3));
  com_task->setMask(Vector3d(1.0, 1.0, 0.0));
  Vector3d com_ref = 0.5 * (robot.framePosition(data, robot.model().getFrameId(LF_FRAME)).translation() +
                            robot.framePosition(data, robot.model().getFrameId(RF_FRAME)).translation());
  com_ref.x() += 0.04; // sole centre is 4 cm ahead of the ankle
  TrajectorySample com_sample(3);
  com_sample.setValue(com_ref);
  com_sample.setDerivative(Vector3d::Zero());
  com_sample.setSecondDerivative(Vector3d::Zero());
  com_task->setReference(com_sample);
  tsid.addMotionTask(*com_task, w_com, 1);

  // Respect the MuJoCo actuator ctrlrange so TSID plans torques that are
  // actually applied (MuJoCo would silently clip them otherwise).
  auto torque_bounds = std::make_shared<TaskActuationBounds>("task-torque-bounds", robot);
  VectorXd tau_max(robot.na());
  for (int i = 0; i < robot.na(); ++i) tau_max(i) = sim.model()->actuator_ctrlrange[2 * i + 1];
  torque_bounds->setBounds(-tau_max, tau_max);
  tsid.addActuationTask(*torque_bounds, 1.0, 0);

  // Posture task for tracking joint-space configuration
  auto posture_task = std::make_shared<TaskJointPosture>("task-posture", robot);

  // Kp and Kd gain vectors for the posture task from YAML config
  VectorXd Kp = VectorXd::Zero(robot.na());
  VectorXd Kd = VectorXd::Zero(robot.na());
  utils::load_postureTask_gain(JOINT_PD_CF_PATH, Kp, Kd);
  // TSID task gains are *acceleration* gains (ddq* = Kp*e + Kd*de), not the
  // Nm/rad joint PD gains in the YAML. The YAML kd values give damping ratios
  // of ~0.05 here, so use critical damping instead.
  Kd = 2.0 * Kp.cwiseSqrt();
  std::cout << "Posture Kp gains:\n" << Kp.transpose() << std::endl;
  std::cout << "Posture Kd gains:\n" << Kd.transpose() << std::endl;

  posture_task->Kp(Kp);
  posture_task->Kd(Kd);

  const double w_posture = 1e-1; // weight in the cost function
  tsid.addMotionTask(*posture_task, w_posture, 1, 0.0);
  std::cout << "Configured posture task successfully." << std::endl;

  // ------------------------------------------------------------------------
  // 4. Initialize HQP Solver (eiquadprog-fast)
  // ------------------------------------------------------------------------
  auto solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                  "QP solver");
  TrajectorySample sample(robot.na());
  VectorXd q_ref(robot.na());
  VectorXd v_ref = VectorXd::Zero(robot.na());
  VectorXd dv_ref = VectorXd::Zero(robot.na());

  // ------------------------------------------------------------------------
  // 5. Simulation Setup & Initial Joint State
  // ------------------------------------------------------------------------
  std::cout << "[SIM] Press '1' to pause/resume; Drag robot body to test compliance.\n";


  // Capture starting joint configuration
  VectorXd q_start = sim.getActuatedJointPos(robot.na());

  // Trajectory parameters: smoothly reach standing pose in 3 seconds
  const double T_stand = 3.0; // seconds

  // Setup CSV log
  DataLogger datalog("record/biped_standing_joint_control.csv");
  vector<string> joint_names = utils::getJointNames(sim.model());
  // MuJoCo joint list is [floating_base_joint, 12 leg joints]
  const vector<string> leg_joint_names(joint_names.end() - robot.na(), joint_names.end());
  datalog.addItem("time", 1);
  datalog.addItem("pos_cmd", leg_joint_names);
  datalog.addItem("pos_measured", leg_joint_names);
  datalog.addItem("tau_cmd", leg_joint_names);
  datalog.finishItemAdding();

  // Real-time joint torque plots, one window per leg. Points are added once
  // per rendered frame (60 Hz) since each line holds at most mjMAXLINEPNT
  // points; 1 kHz samples would only show the last second.
  auto tau_plot_left = std::make_unique<RealtimePlot>(sim.model(), 900, 450, "Left leg torque", 10.0);
  auto tau_plot_right = std::make_unique<RealtimePlot>(sim.model(), 900, 450, "Right leg torque", 10.0);
  for (auto *plot : {tau_plot_left.get(), tau_plot_right.get()}) {
    plot->setYLabel("Nm");
    plot->setLineWidth(2.0f);
  }
  // Legend names: "left_knee_pitch_joint" -> "knee_pitch"
  vector<string> tau_plot_names;
  for (string name : leg_joint_names) {
    for (const string prefix : {"left_", "right_"})
      if (name.rfind(prefix, 0) == 0) name.erase(0, prefix.size());
    if (name.size() > 6 && name.compare(name.size() - 6, 6, "_joint") == 0) name.erase(name.size() - 6);
    tau_plot_names.push_back(name);
  }
  VectorXd tau = VectorXd::Zero(robot.na());

  // ------------------------------------------------------------------------
  // 6. Simulation & Control Loop
  // ------------------------------------------------------------------------
  const double simulation_time = 20.0;
  RobotSensor robot_sensors;

  while (!sim.shouldClose() && sim.time() < simulation_time) {
    double simstart = sim.time();

    // Step physics at ~60 Hz visual frame rate
    while (sim.time() - simstart < 1.0 / 60.0 && sim.runSim) {
      double t = sim.time();

      // Read robot sensors including joint state and imu
      robot_sensors = sim.getRobotSensorValues();
      
      // pass sensor values to state estimator
      const RobotState state = state_estimator.estimate(robot_sensors);
      utils::robotStateToPinocchio(state, q, v);

      // Compute smooth 5th-order polynomial trajectory to qa_init over T_stand
      utils::quinticTrajectory(q_start, qa_init, t, T_stand, q_ref, v_ref, dv_ref);

      sample.setValue(q_ref);
      sample.setDerivative(v_ref);
      sample.setSecondDerivative(dv_ref);
      posture_task->setReference(sample);

      // 6.3 Solve HQP Problem
      const auto &hqpData = tsid.computeProblemData(t, q, v);
      const auto &sol = solver->solve(hqpData);
      if (sol.status != HQP_STATUS_OPTIMAL) {
        std::printf("[TSID] QP failed at t=%.3f (status %d)\n", t, sol.status);
        break;
      }

      // 6.4 Extract torques & apply to MuJoCo actuators via Robot_Simulator
      tau = tsid.getActuatorForces(sol);
      sim.setControl(tau);

      // 6.5 Log state
      datalog.startNewLine();
      datalog.recItemData("time", t);
      datalog.recItemData("pos_cmd", q_ref);
      datalog.recItemData("pos_measured", q.tail(robot.na()));
      datalog.recItemData("tau_cmd", tau);
      datalog.finishLine();

      // 6.6 Apply interactive user perturbations and step physics
      sim.stepPhysics();
    }

    // 6.7 Update torque plots and render frame
    const int n_leg = robot.na() / 2;
    for (int i = 0; i < robot.na(); ++i) {
      RealtimePlot &plot = (i < n_leg) ? *tau_plot_left : *tau_plot_right;
      plot.addPoint(tau_plot_names[i], sim.time(), tau(i));
    }
    tau_plot_left->render();
    tau_plot_right->render();
    sim.updateScene();
  }

  // Flush and close log file
  datalog.close();
  std::cout << "Data saved to " << datalog.path() << " (" << datalog.numLines() << " lines)" << std::endl;

  // Cleanup
  tau_plot_left.reset(); // plot windows must be destroyed before GLFW shuts down
  tau_plot_right.reset();
  sim.Close();
  return 0;
}