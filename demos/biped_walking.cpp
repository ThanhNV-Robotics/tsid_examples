// Standard library
#include <cstdio>
#include <limits>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Third party
#include <Eigen/Dense>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

// Project
#include "sim/Robot_Simulator.h"       // MuJoCo simulator, viewer and RealtimePlot
#include "estimation/cheat_state_estimator.h" // ground-truth state estimator
#include "common/data_logger.h"           // CSV logging
#include "common/data_type.h"             // RobotSensor, RobotState
#include "planning/joystick_interpreter.h"
#include "control/parse_tsid_tasks.h" // builds TSID contacts and tasks from YAML
#include "common/utils.h"            // IK, trajectories, Pinocchio helpers
#include "planning/foot_placement.h"
#include <yaml-cpp/yaml.h>
#include "planning/walk_planner.h"
#include "estimation/contact_fusion.h" // for contact estimation

using namespace tsid;
using namespace tsid::robots;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::trajectories;
using namespace std;
using namespace Eigen;

using tsid::contacts::Contact6d;

int main(int argc, char **argv) {
  const string XML_PATH = std::string(PROJECT_ROOT_DIR) + "/models/biped/mjcf/scene_floatingbase_12dof_v2.xml";
  const string URDF_PATH = std::string(PROJECT_ROOT_DIR) + "/models/biped/urdf/v2_biped_robot_12dof.urdf";
  const string TSID_CONFIG_PATH = std::string(PROJECT_ROOT_DIR) + "/config/tsid_config.yaml";
  const double hip_width = 0.334;
  const double init_base_height = 0.78;
  const double dt = 0.001;

  const double simulation_time = 10.0;
  const double T_stand = 3.0; // seconds, init standing time

  std::printf("====================================================\n");
  std::printf("  Biped TSID Standing Posture Control with MuJoCo   \n");
  std::printf("====================================================\n");

  // ------------------------------------------------------------------------
  // 1. Initialize Robot_Simulator with MuJoCo XML scene
  // ------------------------------------------------------------------------
  Robot_Simulator sim(XML_PATH);
  sim.init("Biped TSID Swing CoM",
           /*saveVideo=*/false); // creates the viewer window
  sim.printModelInfo();

  // ------------------------------------------------------------------------
  // 2. TSID Robot Wrapper
  // ------------------------------------------------------------------------
  std::vector<std::string> package_dirs;
  auto robot_ptr = std::make_shared<RobotWrapper>(
      URDF_PATH, package_dirs, pinocchio::JointModelFreeFlyer(), false);
  RobotWrapper &robot = *robot_ptr;
  // Initialize tsidTaskParser with YAML config and RobotWrapper pointer early

  tsidTaskParser task_parser(TSID_CONFIG_PATH, robot_ptr, URDF_PATH);

  // ------------------------------------------------------------------------
  // 3. Pinocchio model and data for standing IK
  // ------------------------------------------------------------------------
  // Reuse the RobotWrapper's model (with free-flyer); only a Data is needed
  const pinocchio::Model &pin_model = robot.model();
  pinocchio::Data pin_data(pin_model);

  VectorXd qa_init = utils::computeIntialStandConfig(
      pin_model, pin_data, hip_width, init_base_height);
  std::cout << "Target Initial Standing Configuration:\n"
            << qa_init.transpose() << std::endl;

  // Apply initial slightly bent configuration to MuJoCo to avoid kinematic
  // singularity at start
  VectorXd q_bent = VectorXd::Zero(robot.na());
  q_bent[3] = 0.1;    // left knee
  q_bent[9] = 0.1;    // right knee
  q_bent[0] = -0.08;  // left hip pitch
  q_bent[6] = -0.08;  // right hip pitch
  q_bent[5] = -0.05;  // left ankle
  q_bent[11] = -0.08; // right ankle

  // Place the base so that all parsed contact soles rest on the floor
  VectorXd q = task_parser.computeGroundedConfiguration(q_bent);
  VectorXd v = VectorXd::Zero(robot.nv());

  // Set initial configuration in Simulator
  VectorXd qpos_mj(sim.nq());
  qpos_mj << q.head<3>(), 1.0, 0.0, 0.0, 0.0, q_bent; // MuJoCo quat (w,x,y,z)
  sim.setInitConfiguration(qpos_mj);
  std::cout << "Applied initial bent configuration to MuJoCo:\n"
            << qpos_mj.transpose() << std::endl;

  // TSID Formulation and other classes init
  InverseDynamicsFormulationAccForce tsid("tsid-biped", robot);
  Cheat_StateEstimator state_estimator(sim.model(), sim.data());


  utils::robotStateToPinocchio(
      state_estimator.estimate(sim.getRobotSensorValues()), q, v);
  tsid.computeProblemData(0.0, q, v);

  // Configure tasks and update contact placements
  task_parser.setTaskconfig(tsid, robot);
  task_parser.updateContactReferences(tsid.data());

  auto com_task = task_parser.getComTask();
  auto posture_task = task_parser.getPostureTask();

  // Set reference for CoM task horizontally centred over all contact feet
  Vector3d com_ref = task_parser.computeSupportCenter(tsid.data(), robot);
  com_ref.x() += 0.04; // sole centre is 4 cm ahead of the ankle
  com_ref.z() = robot.com(tsid.data()).z(); // hold the current CoM height (the support centre z is the ankle height)
  TrajectorySample com_sample(3);
  com_sample.setValue(com_ref);
  com_sample.setDerivative(Vector3d::Zero());
  com_sample.setSecondDerivative(Vector3d::Zero());
  com_task->setReference(com_sample);

  // ------------------------------------------------------------------------
  // 4. Initialize HQP Solver (eiquadprog-fast)
  // ------------------------------------------------------------------------
  auto solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                  "QP solver");
  TrajectorySample sample(robot.na());
  VectorXd q_ref(robot.na());
  VectorXd v_ref = VectorXd::Zero(robot.na());
  VectorXd dv_ref = VectorXd::Zero(robot.na());


  JoyStickInterpreter joystick(dt);
  // Unified walk planner: CoM / capture point / ZMP in the stance-ankle frame,
  // step timing and touchdown detection, next foothold
  WalkPlanner walk_planner (TSID_CONFIG_PATH, robot.com(tsid.data())[2]);

  // true: the swing foot is lifted (contact removed, swing task tracks the
  // planned trajectory). false: footsteps are only planned and plotted, both
  // feet stay planted.
  const bool enable_swing = true;

  // Contact state used by the walk planner (start of stepping, swing touchdown),
  // and through its step event by the TSID contact switching.
  // true: estimated by ContactFusion (no foot sensor).
  // false: simulator ground truth (MuJoCo contact forces > 20 N).
  const bool use_estimated_contact = true;

  // Walking speed command (YAML "walk_planner: foot_placement:" block)
  const YAML::Node wp_cfg = YAML::LoadFile(TSID_CONFIG_PATH)["walk_planner"];
  const YAML::Node walk_cfg = wp_cfg ? wp_cfg["foot_placement"] : YAML::Node();
  const double vx_cmd = (walk_cfg && walk_cfg["vx"]) ? walk_cfg["vx"].as<double>() : 0.0;
  const double vx_ramp_time = (walk_cfg && walk_cfg["ramp_time"]) ? walk_cfg["ramp_time"].as<double>() : 1.0;
  // Step (swing) duration [s]
  const double Tswing = (walk_cfg && walk_cfg["t_swing"]) ? walk_cfg["t_swing"].as<double>() : 1.0;
  std::printf("[Walking] step duration %.3f s\n", Tswing);
  std::printf("[Walking] vx command %.3f m/s (ramp %.1f s)\n", vx_cmd, vx_ramp_time);

  // Swing-foot planner. The step length follows the vx command (zero command
  // = step in place).
  FootPlacement foot_placement(TSID_CONFIG_PATH, robot);
  foot_placement.openLoopFootsteps = !enable_swing; // chain planned steps while the feet stay planted
  const pinocchio::FrameIndex lf_id = robot.model().getFrameId(foot_placement.leftFootFrame);
  const pinocchio::FrameIndex rf_id = robot.model().getFrameId(foot_placement.rightFootFrame);

  // Contact estimation without a foot sensor: momentum-based disturbance
  // observer -> foot contact wrenches (foot order: left, right). Compared
  // against the simulator's contact forces (Cheat_StateEstimator::getContactForces()).
  ContactFusion contact_estimator(robot.model(),
                                  {foot_placement.leftFootFrame, foot_placement.rightFootFrame}, dt, 15.0);
  // Sole corners for the foot height: the same contact points as the TSID contacts
  for (const auto &info : task_parser.getContactInfos()) {
    if (info.frame_name == foot_placement.leftFootFrame) contact_estimator.setSoleCorners(0, info.contact_points);
    if (info.frame_name == foot_placement.rightFootFrame) contact_estimator.setSoleCorners(1, info.contact_points);
  }

  // ------------------------------------------------------------------------
  // 5. Simulation Setup & Initial Joint State
  // ------------------------------------------------------------------------
  std::cout << "[SIM] Press '1' to pause/resume; Drag robot body to test "
               "compliance.\n";

  // Capture starting joint configuration
  VectorXd q_start = sim.getActuatedJointPos(robot.na());

  // Trajectory parameters: smoothly reach standing pose in 3 seconds

  // Setup CSV log
  DataLogger datalog(std::string(PROJECT_ROOT_DIR) + "/record/biped_standing_joint_control.csv");
  vector<string> joint_names = utils::getJointNames(sim.model());
  // MuJoCo joint list is [floating_base_joint, 12 leg joints]
  const vector<string> leg_joint_names(joint_names.end() - robot.na(),
                                       joint_names.end());
  datalog.addItem("time", 1);
  datalog.addItem("pos_cmd", leg_joint_names);
  datalog.addItem("pos_measured", leg_joint_names);
  datalog.addItem("tau_cmd", leg_joint_names);
  datalog.addItem("com_ref", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("com_meas", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("base_rpy", std::vector<std::string>{"roll", "pitch", "yaw"});
  // swing_leg: 1 = left foot swings, -1 = right foot swings, 0 = double support
  datalog.addItem("swing_leg", 1);
  datalog.addItem("swing_phase", 1);
  datalog.addItem("late_touchdown", 1); // 1 while the walk planner waits for a late touchdown
  datalog.addItem("double_support", 1); // 1 during the double support at the start of a step
  datalog.addItem("step_length", 1);
  datalog.addItem("plan_com", std::vector<std::string>{"x", "y"});
  datalog.addItem("plan_zmp", std::vector<std::string>{"x", "y"});
  datalog.addItem("plan_cp", std::vector<std::string>{"x", "y"}); // planned capture point (world)
  datalog.addItem("meas_cp", std::vector<std::string>{"x", "y"}); // measured capture point c + dc/w (world)
  datalog.addItem("swing_land", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("swing_ref", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("swing_ref_vel", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("lfoot_meas", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("rfoot_meas", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("lfoot_rpy", std::vector<std::string>{"roll", "pitch", "yaw"});
  datalog.addItem("rfoot_rpy", std::vector<std::string>{"roll", "pitch", "yaw"});
  // Contact force (vertical, world z): observer estimate vs. ground truth (MuJoCo contacts)
  datalog.addItem("f_est_z", std::vector<std::string>{"L", "R"});
  datalog.addItem("f_gt_z", std::vector<std::string>{"L", "R"});
  // Sole height (world z of the lowest sole corner), input of the height contact model
  datalog.addItem("sole_z", std::vector<std::string>{"L", "R"});
  datalog.addItem("ground_z", std::vector<std::string>{"L", "R"}); // adaptive ground height under each foot
  // Contact fusion: per-model probabilities, fused probability, estimated and true contact
  datalog.addItem("p_phase", std::vector<std::string>{"L", "R"});
  datalog.addItem("p_height", std::vector<std::string>{"L", "R"});
  datalog.addItem("p_force", std::vector<std::string>{"L", "R"});
  datalog.addItem("p_fused", std::vector<std::string>{"L", "R"});
  datalog.addItem("s_hat", std::vector<std::string>{"L", "R"});
  datalog.addItem("s_gt", std::vector<std::string>{"L", "R"});
  datalog.finishItemAdding();

  // Real-time plots
  // Right leg joint torques (commanded), one line per joint
  // auto tau_right_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
  //                                                      "Right leg torque", 10.0);
  // tau_right_plot->setYLabel("Nm");
  // tau_right_plot->setLineWidth(2.0);

  auto foot_height_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                          "Foot height (contact fusion)", 10.0);
  foot_height_plot->setYLabel("Sole z (m)");
  foot_height_plot->setLineWidth(2.0);

  // // Left leg joint torques (commanded), one line per joint
  // auto tau_left_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
  //                                                     "Left leg torque", 10.0);
  // tau_left_plot->setYLabel("Nm");
  // tau_left_plot->setLineWidth(2.0);
  // Legend names: "left_knee_pitch_joint" -> "knee_pitch"
  vector<string> tau_left_names;
  for (int i = 0; i < robot.na() / 2; ++i) {
    string name = leg_joint_names[i];
    if (name.rfind("left_", 0) == 0) name.erase(0, 5); // same joint names for both legs
    if (name.size() > 6 && name.compare(name.size() - 6, 6, "_joint") == 0) name.erase(name.size() - 6);
    tau_left_names.push_back(name);
  }

  auto phase_contact_prob_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                                 "Phase contact probability", 10.0);
  phase_contact_prob_plot->setYLabel("P(c | s_phi, phi)");
  phase_contact_prob_plot->setLineWidth(2.0);

  auto force_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                   "Foot contact force", 10.0);
  force_plot->setYLabel("Fz (N)");
  force_plot->setLineWidth(2.0);

  auto force_contact_prob_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                                 "Force contact probability", 10.0);
  force_contact_prob_plot->setYLabel("P(c | f_z)");
  force_contact_prob_plot->setLineWidth(2.0);

  auto height_contact_prob_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                                  "Height contact probability", 10.0);
  height_contact_prob_plot->setYLabel("P(c | p_z)");
  height_contact_prob_plot->setLineWidth(2.0);

  auto fused_contact_prob_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                                 "Fused contact probability (Kalman)", 10.0);
  fused_contact_prob_plot->setYLabel("P(c)");
  fused_contact_prob_plot->setLineWidth(2.0);

  auto contact_state_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                            "Contact state: estimated vs ground truth", 10.0);
  contact_state_plot->setYLabel("contact (L: 0/1, R: 1.5/2.5)");
  contact_state_plot->setLineWidth(2.0);
  // Contact fusion accuracy (estimated vs. simulator contact state) while walking
  long n_fusion_ticks = 0;
  long n_fusion_correct[2] = {0, 0};

  VectorXd tau = VectorXd::Zero(robot.na());

  // ------------------------------------------------------------------------
  // 6. Simulation & Control Loop
  // ------------------------------------------------------------------------
  RobotSensor robot_sensors;
  bool startWalking = false;
  bool qp_failed = false;
  std::string swing_frame;          // frame of the foot currently in the air, empty in double support
  bool was_late = false;            // walk planner in the late-touchdown state on the previous tick
  Matrix3d swing_R = Matrix3d::Identity(); // swing foot orientation target during the swing
  // Nominal foot orientations (flat, standing yaw) captured when walking
  // starts; every swing returns the foot to it, so yaw slip of the stance foot
  // cannot accumulate over the steps
  Matrix3d R_nominal_lf = Matrix3d::Identity(), R_nominal_rf = Matrix3d::Identity();

  // while (!sim.shouldClose() && !qp_failed && sim.time() <= simulation_time) 
  // TSID_SIM_TIME=<seconds>: stop at that simulated time (batch / headless runs)
  const char *sim_time_env = std::getenv("TSID_SIM_TIME");
  const double max_sim_time = sim_time_env ? std::atof(sim_time_env) : std::numeric_limits<double>::infinity();
  while (!sim.shouldClose() && !qp_failed && sim.time() <= max_sim_time) 
  {
    double simstart = sim.time();

    // Step physics at ~60 Hz visual frame rate
    while (sim.time() - simstart < 1.0 / 60.0 && sim.runSim) {
      double t = sim.time();

      // Read robot sensors including joint state and imu
      robot_sensors = sim.getRobotSensorValues();
      // pass sensor values to state estimator
      const RobotState state = state_estimator.estimate(robot_sensors);
      utils::robotStateToPinocchio(state, q, v);
      contact_estimator.computeDisturbanceTorque(q, v, robot_sensors.actuator_state.torquej);

      // Contact fusion (used by the walk planner if use_estimated_contact; always
      // logged against the ground truth). Gait schedule from the walk planner's previous
      // tick: the stance leg is scheduled in contact, the swing leg is not; both
      // feet in double support (standing / init shift) with subphase 0.5
      {
        // Double support (start of each step): both feet scheduled in contact
        const LegState stance = walk_planner.getStanceLeg();
        const bool both = (stance == LegState::DSt) || walk_planner.inDoubleSupport();
        const double phi = both ? 0.5 : walk_planner.getSwingPhase();
        contact_estimator.fuseContactProbability({both || stance != LegState::RSt, both || stance != LegState::LSt},
                                                 {phi, phi});
        // Accuracy against the simulator's contacts, while walking
        if (t >= T_stand) {
          ++n_fusion_ticks;
          for (int i = 0; i < 2; ++i)
            if (contact_estimator.getContactState(i) == state.contact_flags[i]) ++n_fusion_correct[i];
        }
      }

      // ------------------------------------------------------------------------
      // Initial standing task
      // ------------------------------------------------------------------------
      if (t <= T_stand) {
        // (x, y, z, yaw): the 3-argument overload is (x, y, yaw)
        const Matrix3d R_base = state.quat_b_W.toRotationMatrix();
        joystick.setIniPos(state.pos_b_W[0], state.pos_b_W[1], state.pos_b_W[2],
                           std::atan2(R_base(1, 0), R_base(0, 0)));
        joystick.setVxDesLPara(0, 0.1);
        joystick.setWzDesLPara(0, 0.1);
        joystick.setPzRef(state.pos_b_W[2], 0.1);
      }

      if (t >= T_stand && !startWalking)
      {
        startWalking = true;
        joystick.setMotionState(MotionState::WALK);
        joystick.setVxDesLPara(vx_cmd, vx_ramp_time);
        // tsid.removeTask(posture_task->name(),2.5);
        // The walk planner first shifts the CoM onto the left foot with both
        // feet down, then starts stepping with the left leg as stance (the
        // right foot swings first)

        // Nominal stance width = the actual distance between the planted feet,
        // used by both planners so the lateral targets lie on the feet
        foot_placement.stanceWidth =
            (robot.framePosition(tsid.data(), lf_id).translation() - robot.framePosition(tsid.data(), rf_id).translation())
                .head<2>().norm();
        walk_planner.wd_hip = foot_placement.stanceWidth;
        // Nominal foot orientations: flat (zero roll/pitch) with the standing yaw
        for (auto [fid, R_nom] : {std::pair<pinocchio::FrameIndex, Matrix3d *>{lf_id, &R_nominal_lf},
                                  std::pair<pinocchio::FrameIndex, Matrix3d *>{rf_id, &R_nominal_rf}}) {
          const Matrix3d Rf = robot.framePosition(tsid.data(), fid).rotation();
          *R_nom = Eigen::AngleAxisd(std::atan2(Rf(1, 0), Rf(0, 0)), Vector3d::UnitZ()).toRotationMatrix();
        }
        std::printf("[FootPlacement] stance width %.3f m\n", foot_placement.stanceWidth);
      }

      // ------------------------------------------------------------------------
      // swinging CoM task
      // ------------------------------------------------------------------------
      if (startWalking)
      { 
        std::vector<bool> contact_flags;
        if (use_estimated_contact)
        {
          // get contact flags/event from contact fusion model
          contact_flags = std::vector<bool>{contact_estimator.getContactState(0), contact_estimator.getContactState(1)}; 
        }
        else 
        {
          // get contact flags event from mujoco (cheat estimator)
          contact_flags = state.contact_flags; 
        }
        
        walk_planner.planWalking(joystick, state_estimator, robot, tsid.data(), Tswing, contact_flags);
        // CoM reference (world frame) with the planned velocity and acceleration
        // as feedforward (zero derivatives would make the Kd term brake the motion)
        com_sample.setValue(walk_planner.getCoMrefW());
        com_sample.setDerivative(walk_planner.getCoMvelRefW());
        com_sample.setSecondDerivative(walk_planner.getCoMaccRefW());
        com_task->setReference(com_sample);

        // Swing-foot trajectory: swing leg, phase and landing point from the
        // walk planner. Uses the kinematics of the previous step.
        foot_placement.StepSwingPlanning(state, tsid.data(), joystick, walk_planner);

        if (enable_swing) {
          // Contact switch on the walk planner's step event. The planner
          // detects the touchdown (swing foot back in contact late enough in
          // the step, or the step timing out if it never lifted), so the
          // landing foot gets its contact back and the other foot lifts off
          // Late touchdown: the step is over but the swing foot has not found the
          // ground; the planner keeps the support leg while the foot descends
          if (walk_planner.isLate() && !was_late) {
            std::printf("[Swing] t=%.3f late touchdown: %s not in contact at the end of the step\n", t,
                        swing_frame.c_str());
          }
          was_late = walk_planner.isLate();
          // Touchdown event: the landing foot gets its rigid contact back; the
          // step starts with a double support (both feet in contact)
          if (walk_planner.stanceChanged()) {
            if (!swing_frame.empty() && task_parser.isSwinging(swing_frame))
            {
              task_parser.endSwing(tsid, swing_frame, tsid.data()); // add rigid contact for the landing foot
              std::printf("[Swing] t=%.3f touchdown %s\n", t, swing_frame.c_str());
            }
            swing_frame.clear();
          }
          // Liftoff event (end of the double support): the trailing foot lifts off
          if (walk_planner.liftoffEvent()) {
            swing_frame = foot_placement.getSwingFrameName();
            swing_R = foot_placement.isLeftSwing() ? R_nominal_lf : R_nominal_rf;
            task_parser.startSwing(tsid, swing_frame);
            std::printf("[Swing] t=%.3f liftoff %s\n", t, swing_frame.c_str());
          }
          if (!swing_frame.empty() && task_parser.isSwinging(swing_frame)) {
            task_parser.setSwingReference(swing_frame, foot_placement.getSwingDesPos(),
                                          foot_placement.getSwingDesVel(), foot_placement.getSwingDesAcc(), swing_R);
          }
        }
      }

      joystick.step();

      if (t<= T_stand)
      {
        // Compute smooth 5th-order polynomial trajectory to qa_init over
        // T_stand
        utils::quinticTrajectory(q_start, qa_init, t, T_stand, q_ref, v_ref,
                                 dv_ref);
        sample.setValue(q_ref);
        sample.setDerivative(v_ref);
        sample.setSecondDerivative(dv_ref);
        posture_task->setReference(sample);
      }
      

      // 6.3 Solve HQP Problem
      const auto &hqpData = tsid.computeProblemData(t, q, v);
      const auto &sol = solver->solve(hqpData);
      if (sol.status != HQP_STATUS_OPTIMAL) {
        std::printf("[TSID] QP failed at t=%.3f (status %d), stopping\n", t, sol.status);
        qp_failed = true;
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
      datalog.recItemData("com_ref", com_sample.getValue());
      datalog.recItemData("com_meas", robot.com(tsid.data()));
      {
        const Matrix3d Rb = state.quat_b_W.toRotationMatrix(); // roll-pitch-yaw (ZYX)
        datalog.recItemData("base_rpy", Vector3d(std::atan2(Rb(2, 1), Rb(2, 2)), std::asin(-Rb(2, 0)),
                                                 std::atan2(Rb(1, 0), Rb(0, 0))));
      }
      datalog.recItemData("swing_leg", !foot_placement.isSwinging() ? 0.0
                                       : foot_placement.isLeftSwing() ? 1.0 : -1.0);
      datalog.recItemData("swing_phase", foot_placement.phi);
      datalog.recItemData("late_touchdown", walk_planner.isLate() ? 1.0 : 0.0);
      datalog.recItemData("double_support", walk_planner.inDoubleSupport() ? 1.0 : 0.0);
      datalog.recItemData("step_length", foot_placement.stepLength);
      datalog.recItemData("plan_com", Vector2d(walk_planner.getCoMrefW().head<2>()));
      datalog.recItemData("plan_zmp", Vector2d(walk_planner.getZMPrefW().head<2>()));
      datalog.recItemData("plan_cp", Vector2d(walk_planner.getCapturedPointrefW().head<2>()));
      datalog.recItemData("meas_cp", Vector2d(walk_planner.getMeasuredCPW().head<2>()));
      datalog.recItemData("swing_land", foot_placement.posDes_W);
      datalog.recItemData("swing_ref", foot_placement.getSwingDesPos());
      datalog.recItemData("swing_ref_vel", foot_placement.getSwingDesVel());
      datalog.recItemData("lfoot_meas", robot.framePosition(tsid.data(), lf_id).translation());
      datalog.recItemData("rfoot_meas", robot.framePosition(tsid.data(), rf_id).translation());
      for (const auto &[item, fid] : {std::pair<const char *, pinocchio::FrameIndex>{"lfoot_rpy", lf_id},
                                      std::pair<const char *, pinocchio::FrameIndex>{"rfoot_rpy", rf_id}}) {
        const Matrix3d Rf = robot.framePosition(tsid.data(), fid).rotation(); // roll-pitch-yaw (ZYX)
        datalog.recItemData(item, Vector3d(std::atan2(Rf(2, 1), Rf(2, 2)), std::asin(-Rf(2, 0)),
                                           std::atan2(Rf(1, 0), Rf(0, 0))));
      }
      datalog.recItemData("f_est_z", Vector2d(contact_estimator.getFootWrench(0)(2),
                                              contact_estimator.getFootWrench(1)(2)));
      datalog.recItemData("f_gt_z", Vector2d(state_estimator.getContactForces()[0].z(),
                                             state_estimator.getContactForces()[1].z()));
      datalog.recItemData("sole_z", Vector2d(contact_estimator.getFootHeight(0), contact_estimator.getFootHeight(1)));
      datalog.recItemData("ground_z", Vector2d(contact_estimator.getGroundHeight(0), contact_estimator.getGroundHeight(1)));
      datalog.recItemData("p_phase", Vector2d(contact_estimator.getPhaseProbability(0), contact_estimator.getPhaseProbability(1)));
      datalog.recItemData("p_height", Vector2d(contact_estimator.getHeightProbability(0), contact_estimator.getHeightProbability(1)));
      datalog.recItemData("p_force", Vector2d(contact_estimator.getForceProbability(0), contact_estimator.getForceProbability(1)));
      datalog.recItemData("p_fused", Vector2d(contact_estimator.getContactProbability(0), contact_estimator.getContactProbability(1)));
      datalog.recItemData("s_hat", Vector2d(contact_estimator.getContactState(0), contact_estimator.getContactState(1)));
      datalog.recItemData("s_gt", Vector2d(state.contact_flags[0], state.contact_flags[1]));
      datalog.finishLine();
      // 6.6 Apply interactive user perturbations and step physics
      sim.stepPhysics();
    }
    // // right leg joints follow the 6 left leg joints in tau
    // for (int i = 0; i < robot.na() / 2; ++i) {
    //   tau_right_plot->addPoint(tau_left_names[i], sim.time(), tau(robot.na() / 2 + i));
    // }
    // tau_right_plot->render();

    // for (int i = 0; i < robot.na() / 2; ++i) {
    //   tau_left_plot->addPoint(tau_left_names[i], sim.time(), tau(i));
    // }
    // tau_left_plot->render();

    // world frame, so the reference and the measured CoM are comparable
    // Phase-based contact probability (paper Sec. IV-A) of each foot. Scheduled
    // state: the stance leg is in contact, the swing leg is not; both feet are
    // in contact in double support (standing / init shift). Subphase: the step
    // phase for both feet, 0.5 (mid-stance) in double support.
    {
      const LegState stance = walk_planner.getStanceLeg();
      const bool both = (stance == LegState::DSt) || walk_planner.inDoubleSupport();
      const bool s_L = both || (stance != LegState::RSt);
      const bool s_R = both || (stance != LegState::LSt);
      phase_contact_prob_plot->addPoint("P_L", sim.time(), contact_estimator.getPhaseProbability(0));
      phase_contact_prob_plot->addPoint("P_R", sim.time(), contact_estimator.getPhaseProbability(1));
      phase_contact_prob_plot->addPoint("s_L", sim.time(), s_L ? 1.0 : 0.0);
      phase_contact_prob_plot->addPoint("s_R", sim.time(), s_R ? 1.0 : 0.0);
      phase_contact_prob_plot->render();
    }

    // Sole height used by the height contact model: world z of the lowest sole corner
    foot_height_plot->addPoint("z_L", sim.time(), contact_estimator.getFootHeight(0));
    foot_height_plot->addPoint("z_R", sim.time(), contact_estimator.getFootHeight(1));
    // Adaptive ground height estimated under each foot
    foot_height_plot->addPoint("ground_L", sim.time(), contact_estimator.getGroundHeight(0));
    foot_height_plot->addPoint("ground_R", sim.time(), contact_estimator.getGroundHeight(1));
    foot_height_plot->render();

    // Estimated vs. ground-truth (MuJoCo contacts) vertical force of each foot
    force_plot->addPoint("L_est", sim.time(), contact_estimator.getFootWrench(0)(2));
    force_plot->addPoint("L_gt", sim.time(), state_estimator.getContactForces()[0].z());
    force_plot->addPoint("R_est", sim.time(), contact_estimator.getFootWrench(1)(2));
    force_plot->addPoint("R_gt", sim.time(), state_estimator.getContactForces()[1].z());
    force_plot->render();

    // Force-based contact probability (paper eq. 22) of each foot, from the
    // vertical force estimated by the disturbance observer, against the
    // simulator's ground-truth contact flags (vertical contact force > 20 N)
    {
      const std::vector<bool> &gt = state_estimator.getState().contact_flags;
      force_contact_prob_plot->addPoint("P_L", sim.time(), contact_estimator.getForceProbability(0));
      force_contact_prob_plot->addPoint("P_R", sim.time(), contact_estimator.getForceProbability(1));
      force_contact_prob_plot->addPoint("gt_L", sim.time(), gt[0] ? 1.0 : 0.0);
      force_contact_prob_plot->addPoint("gt_R", sim.time(), gt[1] ? 1.0 : 0.0);
      force_contact_prob_plot->render();

      // Height-based contact probability (paper eq. 20) from the lowest sole corner
      height_contact_prob_plot->addPoint("P_L", sim.time(), contact_estimator.getHeightProbability(0));
      height_contact_prob_plot->addPoint("P_R", sim.time(), contact_estimator.getHeightProbability(1));
      height_contact_prob_plot->addPoint("gt_L", sim.time(), gt[0] ? 1.0 : 0.0);
      height_contact_prob_plot->addPoint("gt_R", sim.time(), gt[1] ? 1.0 : 0.0);
      height_contact_prob_plot->render();

      // Kalman-fused contact probability vs. the simulator's contact state
      fused_contact_prob_plot->addPoint("P_L", sim.time(), contact_estimator.getContactProbability(0));
      fused_contact_prob_plot->addPoint("P_R", sim.time(), contact_estimator.getContactProbability(1));
      fused_contact_prob_plot->addPoint("gt_L", sim.time(), gt[0] ? 1.0 : 0.0);
      fused_contact_prob_plot->addPoint("gt_R", sim.time(), gt[1] ? 1.0 : 0.0);
      fused_contact_prob_plot->render();

      // Estimated contact state s_hat (hysteresis on the fused probability) vs.
      // the simulator's contact state; right foot shifted by 1.5 so the feet
      // do not overlap
      contact_state_plot->addPoint("s_hat_L", sim.time(), contact_estimator.getContactState(0) ? 1.0 : 0.0);
      contact_state_plot->addPoint("gt_L", sim.time(), gt[0] ? 1.0 : 0.0);
      contact_state_plot->addPoint("s_hat_R", sim.time(), contact_estimator.getContactState(1) ? 2.5 : 1.5);
      contact_state_plot->addPoint("gt_R", sim.time(), gt[1] ? 2.5 : 1.5);
      contact_state_plot->render();
    }

    // Swing-foot plan in the viewer: a green sphere at the planned swing-foot
    // position with blue spheres at its liftoff and landing points
    // if (startWalking) {
    //   if (foot_placement.isSwinging()) {
    //     const float green[4] = {0.2f, 0.9f, 0.3f, 1.0f};
    //     const float blue[4] = {0.2f, 0.5f, 1.0f, 0.8f};
    //     sim.addSphere(foot_placement.getSwingDesPos(), 0.02, green);
    //     sim.addSphere(foot_placement.posStart_W, 0.012, blue);
    //     sim.addSphere(foot_placement.posDes_W, 0.012, blue);
    //   }
    // }

    // Yellow: measured CoM. Red: desired ZMP of the CP planner on the floor,
    // with a line from the CoM (the ground reaction force direction in the
    // LIPM). Before walking starts, the CoM's ground projection is shown.
    const float yellow[4] = {1.0f, 0.85f, 0.1f, 1.0f};
    const float red[4] = {1.0f, 0.15f, 0.15f, 1.0f};
    const Vector3d com_meas = sim.getCoM();
    sim.addSphere(com_meas, 0.025, yellow);
    if (startWalking) {
      const Vector3d zmp_des = walk_planner.getZMPrefW();
      sim.addLine(com_meas, zmp_des, red, 2.0);
      sim.addSphere(zmp_des, 0.015, red);
    } else {
      const Vector3d com_ground(com_meas.x(), com_meas.y(), 0.0);
      sim.addLine(com_meas, com_ground, yellow, 2.0);
      sim.addSphere(com_ground, 0.012, yellow);
    }
    sim.updateScene();    
  }

  // Flush and close log file
  datalog.close();
  if (n_fusion_ticks > 0)
    std::printf("[ContactFusion] contact state accuracy while walking: left %.2f %%, right %.2f %% (%ld ticks)\n",
                100.0 * n_fusion_correct[0] / n_fusion_ticks, 100.0 * n_fusion_correct[1] / n_fusion_ticks,
                n_fusion_ticks);
  std::cout << "Data saved to " << datalog.path() << " (" << datalog.numLines()
            << " lines)" << std::endl;

  // plot windows must be destroyed before GLFW shuts down
  // tau_right_plot.reset();
  // tau_left_plot.reset();
  phase_contact_prob_plot.reset();
  force_contact_prob_plot.reset();
  height_contact_prob_plot.reset();
  fused_contact_prob_plot.reset();
  contact_state_plot.reset();
  force_plot.reset();
  foot_height_plot.reset();
  sim.Close();
  return 0;
}