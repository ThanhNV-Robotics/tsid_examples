// Standard library
#include <cstdio>
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
#include "Robot_Simulator.h"       // MuJoCo simulator, viewer and RealtimePlot
#include "cheat_state_estimator.h" // ground-truth state estimator
#include "data_logger.h"           // CSV logging
#include "data_type.h"             // RobotSensor, RobotState
#include "joystick_interpreter.h"
#include "my_gait_scheduler.h"
#include "parse_tsid_tasks.h" // builds TSID contacts and tasks from YAML
#include "utils.h"            // IK, trajectories, Pinocchio helpers
#include "CP_Planning.h"
#include "foot_placement.h"
#include <yaml-cpp/yaml.h>

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
  const string TSID_CONFIG_PATH = "config/tsid_config.yaml";
  const double hip_width = 0.334;
  const double init_base_height = 0.78;
  const double dt = 0.001;
  const double Tswing = 1.0;

  const double simulation_time = 15.0;
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
  const pinocchio::Data &data = tsid.data();

  // Configure tasks and update contact placements
  task_parser.setTaskconfig(tsid, robot);
  task_parser.updateContactReferences(data);

  auto com_task = task_parser.getComTask();
  auto posture_task = task_parser.getPostureTask();

  // Set reference for CoM task horizontally centred over all contact feet
  Vector3d com_ref = task_parser.computeSupportCenter(data, robot);
  com_ref.x() += 0.04; // sole centre is 4 cm ahead of the ankle
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
  MyGaitScheduler gait_scheduler(Tswing, dt);
  CP_Planning cp_planner (dt, robot.com(data)[2], hip_width);

  // true: the swing foot is lifted (contact removed, swing task tracks the
  // planned trajectory). false: footsteps are only planned and plotted, both
  // feet stay planted.
  const bool enable_swing = true;

  // Walking speed command (YAML "walking_command:" block)
  const YAML::Node walk_cfg = YAML::LoadFile(TSID_CONFIG_PATH)["walking_command"];
  const double vx_cmd = (walk_cfg && walk_cfg["vx"]) ? walk_cfg["vx"].as<double>() : 0.0;
  const double vx_ramp_time = (walk_cfg && walk_cfg["ramp_time"]) ? walk_cfg["ramp_time"].as<double>() : 1.0;
  std::printf("[Walking] vx command %.3f m/s (ramp %.1f s)\n", vx_cmd, vx_ramp_time);

  // Swing-foot planner. The step length follows the vx command (zero command
  // = step in place).
  FootPlacement foot_placement(TSID_CONFIG_PATH, robot);
  foot_placement.openLoopFootsteps = !enable_swing; // chain planned steps while the feet stay planted
  cp_planner.max_step_length = foot_placement.maxStepLength;
  const pinocchio::FrameIndex lf_id = robot.model().getFrameId(foot_placement.leftFootFrame);
  const pinocchio::FrameIndex rf_id = robot.model().getFrameId(foot_placement.rightFootFrame);

  // ------------------------------------------------------------------------
  // 5. Simulation Setup & Initial Joint State
  // ------------------------------------------------------------------------
  std::cout << "[SIM] Press '1' to pause/resume; Drag robot body to test "
               "compliance.\n";

  // Capture starting joint configuration
  VectorXd q_start = sim.getActuatedJointPos(robot.na());

  // Trajectory parameters: smoothly reach standing pose in 3 seconds

  // Setup CSV log
  DataLogger datalog("record/biped_standing_joint_control.csv");
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
  datalog.addItem("step_length", 1);
  datalog.addItem("plan_com", std::vector<std::string>{"x", "y"});
  datalog.addItem("plan_zmp", std::vector<std::string>{"x", "y"});
  datalog.addItem("swing_land", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("swing_ref", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("swing_ref_vel", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("lfoot_meas", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("rfoot_meas", std::vector<std::string>{"x", "y", "z"});
  datalog.addItem("lfoot_rpy", std::vector<std::string>{"roll", "pitch", "yaw"});
  datalog.addItem("rfoot_rpy", std::vector<std::string>{"roll", "pitch", "yaw"});
  datalog.finishItemAdding();

  // Real-time plots
  // Right leg joint torques (commanded), one line per joint
  auto tau_right_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                       "Right leg torque", 10.0);
  tau_right_plot->setYLabel("Nm");
  tau_right_plot->setLineWidth(2.0);

  // Left leg joint torques (commanded), one line per joint
  auto tau_left_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                      "Left leg torque", 10.0);
  tau_left_plot->setYLabel("Nm");
  tau_left_plot->setLineWidth(2.0);
  // Legend names: "left_knee_pitch_joint" -> "knee_pitch"
  vector<string> tau_left_names;
  for (int i = 0; i < robot.na() / 2; ++i) {
    string name = leg_joint_names[i];
    if (name.rfind("left_", 0) == 0) name.erase(0, 5); // same joint names for both legs
    if (name.size() > 6 && name.compare(name.size() - 6, 6, "_joint") == 0) name.erase(name.size() - 6);
    tau_left_names.push_back(name);
  }

  auto cp_planner_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                      "Capture Point Planner", 10.0);
  cp_planner_plot->setYLabel("CoM X/Y");
  cp_planner_plot->setLineWidth(2.0);

  auto foot_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                  "Swing Foot Planner", 10.0);
  foot_plot->setYLabel("Foot Z (m)");
  foot_plot->setLineWidth(2.0);

  VectorXd tau = VectorXd::Zero(robot.na());

  // ------------------------------------------------------------------------
  // 6. Simulation & Control Loop
  // ------------------------------------------------------------------------
  RobotSensor robot_sensors;
  bool startWalking = false;
  bool qp_failed = false;
  std::string swing_frame;          // frame of the foot currently in the air, empty in double support
  Matrix3d swing_R = Matrix3d::Identity(); // swing foot orientation target during the swing
  // Nominal foot orientations (flat, standing yaw) captured when walking
  // starts; every swing returns the foot to it, so yaw slip of the stance foot
  // cannot accumulate over the steps
  Matrix3d R_nominal_lf = Matrix3d::Identity(), R_nominal_rf = Matrix3d::Identity();
  while (!sim.shouldClose() && !qp_failed) {
    double simstart = sim.time();

    // Step physics at ~60 Hz visual frame rate
    while (sim.time() - simstart < 1.0 / 60.0 && sim.runSim) {
      double t = sim.time();

      // Read robot sensors including joint state and imu
      robot_sensors = sim.getRobotSensorValues();
      // pass sensor values to state estimator
      const RobotState state = state_estimator.estimate(robot_sensors);
      utils::robotStateToPinocchio(state, q, v);

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
        // First cycle shifts the CoM toward the left foot (RSt target) with no
        // swing; the right foot swings first, in the second cycle
        gait_scheduler.firstleg = LegState::RSt;
        gait_scheduler.start(joystick);
        cp_planner.setInitCom(robot.com(data));
        // Nominal stance width = the actual distance between the planted feet,
        // used by both planners so the lateral targets lie on the feet
        foot_placement.stanceWidth =
            (robot.framePosition(data, lf_id).translation() - robot.framePosition(data, rf_id).translation())
                .head<2>().norm();
        cp_planner.wd_hip = foot_placement.stanceWidth;
        // Nominal foot orientations: flat (zero roll/pitch) with the standing yaw
        for (auto [fid, R_nom] : {std::pair<pinocchio::FrameIndex, Matrix3d *>{lf_id, &R_nominal_lf},
                                  std::pair<pinocchio::FrameIndex, Matrix3d *>{rf_id, &R_nominal_rf}}) {
          const Matrix3d Rf = robot.framePosition(data, fid).rotation();
          *R_nom = Eigen::AngleAxisd(std::atan2(Rf(1, 0), Rf(0, 0)), Vector3d::UnitZ()).toRotationMatrix();
        }
        std::printf("[FootPlacement] stance width %.3f m\n", foot_placement.stanceWidth);
      }

      // ------------------------------------------------------------------------
      // swinging CoM task
      // ------------------------------------------------------------------------
      if (startWalking)
      {
        gait_scheduler.step(state_estimator);
        cp_planner.planWalking(gait_scheduler, joystick);
        // CoM reference with the planned velocity and acceleration as
        // feedforward (zero derivatives would make the Kd term brake the motion)
        com_sample.setValue(cp_planner.getCoMref());
        com_sample.setDerivative(cp_planner.getCoMvelRef());
        com_sample.setSecondDerivative(cp_planner.getCoMaccRef());
        com_task->setReference(com_sample);

        // Swing-foot plan; swing leg and phase from the CP planner (phi_swing,
        // one cycle behind the CoM). Uses the kinematics of the previous step.
        foot_placement.StepSwingPlanning(state, tsid.data(), joystick, cp_planner);

        if (enable_swing) {
          // Contact switch whenever the swinging foot changes: the landing
          // foot gets its contact back, the lifting foot loses it
          const std::string new_swing = foot_placement.isSwinging() ? foot_placement.getSwingFrameName() : "";
          if (new_swing != swing_frame) {
            if (!swing_frame.empty() && task_parser.isSwinging(swing_frame)) {
              task_parser.endSwing(tsid, swing_frame, tsid.data());
              std::printf("[Swing] t=%.3f touchdown %s (end of phase)\n", t, swing_frame.c_str());
            }
            if (!new_swing.empty()) {
              swing_R = foot_placement.isLeftSwing() ? R_nominal_lf : R_nominal_rf;
              task_parser.startSwing(tsid, new_swing);
              std::printf("[Swing] t=%.3f liftoff %s\n", t, new_swing.c_str());
            }
            swing_frame = new_swing;
          }
          // Early touchdown: once past mid-swing, plant the foot as soon as its
          // touch sensor reports contact instead of pushing it further toward
          // the stretched (below-ground) target until the phase ends
          if (!swing_frame.empty() && task_parser.isSwinging(swing_frame) && foot_placement.phi > 0.5) {
            const bool touching = foot_placement.isLeftSwing() ? state.contact_flags[0] : state.contact_flags[1];
            if (touching) {
              task_parser.endSwing(tsid, swing_frame, tsid.data());
              std::printf("[Swing] t=%.3f touchdown %s (early, phase %.2f)\n", t, swing_frame.c_str(),
                          foot_placement.phi);
            }
          }
          if (!swing_frame.empty() && task_parser.isSwinging(swing_frame)) {
            task_parser.setSwingReference(swing_frame, foot_placement.getSwingDesPos(),
                                          foot_placement.getSwingDesVel(), foot_placement.getSwingDesAcc(), swing_R);
          }
        }
      }

      joystick.step();
      // Compute smooth 5th-order polynomial trajectory to qa_init over T_stand
      utils::quinticTrajectory(q_start, qa_init, t, T_stand, q_ref, v_ref,
                               dv_ref);
      sample.setValue(q_ref);
      sample.setDerivative(v_ref);
      sample.setSecondDerivative(dv_ref);
      posture_task->setReference(sample);

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
      datalog.recItemData("step_length", foot_placement.stepLength);
      datalog.recItemData("plan_com", Vector2d(cp_planner.xc_, cp_planner.yc_));
      datalog.recItemData("plan_zmp", Vector2d(cp_planner.px_d_, cp_planner.py_d_));
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
      datalog.finishLine();
      // 6.6 Apply interactive user perturbations and step physics
      sim.stepPhysics();
    }
    // right leg joints follow the 6 left leg joints in tau
    for (int i = 0; i < robot.na() / 2; ++i) {
      tau_right_plot->addPoint(tau_left_names[i], sim.time(), tau(robot.na() / 2 + i));
    }
    tau_right_plot->render();

    for (int i = 0; i < robot.na() / 2; ++i) {
      tau_left_plot->addPoint(tau_left_names[i], sim.time(), tau(i));
    }
    tau_left_plot->render();

    cp_planner_plot->addPoint("ZMP_Y", sim.time(), cp_planner.py_d_);
    cp_planner_plot->addPoint("CP_Y", sim.time(), cp_planner.cxi_y_);
    cp_planner_plot->addPoint("CoM_Y_ref", sim.time(), cp_planner.yc_);
    cp_planner_plot->addPoint("CoM_Y_fb", sim.time(), robot.com(data)[1]);
    cp_planner_plot->render();

    // Swing-foot plan: planned foot height vs. the measured foot heights, and
    // in the viewer a green sphere at the planned swing-foot position with
    // blue spheres at its liftoff and landing points
    if (startWalking) {
      foot_plot->addPoint("Swing_Z_ref", sim.time(), foot_placement.getSwingDesPos().z());
      foot_plot->addPoint("LFoot_Z", sim.time(), robot.framePosition(data, lf_id).translation().z());
      foot_plot->addPoint("RFoot_Z", sim.time(), robot.framePosition(data, rf_id).translation().z());
      foot_plot->addPoint("Swing_Leg(L=+0.1)", sim.time(),
                          !foot_placement.isSwinging() ? 0.0 : foot_placement.isLeftSwing() ? 0.1 : -0.1);
      foot_plot->render();

      if (foot_placement.isSwinging()) {
        const float green[4] = {0.2f, 0.9f, 0.3f, 1.0f};
        const float blue[4] = {0.2f, 0.5f, 1.0f, 0.8f};
        sim.addSphere(foot_placement.getSwingDesPos(), 0.02, green);
        sim.addSphere(foot_placement.posStart_W, 0.012, blue);
        sim.addSphere(foot_placement.posDes_W, 0.012, blue);
      }
    }

    // Yellow: measured CoM. Red: desired ZMP of the CP planner on the floor,
    // with a line from the CoM (the ground reaction force direction in the
    // LIPM). Before walking starts, the CoM's ground projection is shown.
    const float yellow[4] = {1.0f, 0.85f, 0.1f, 1.0f};
    const float red[4] = {1.0f, 0.15f, 0.15f, 1.0f};
    const Vector3d com_meas = sim.getCoM();
    sim.addSphere(com_meas, 0.025, yellow);
    if (startWalking) {
      const Vector3d zmp_des(cp_planner.px_d_, cp_planner.py_d_, 0.0);
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
  std::cout << "Data saved to " << datalog.path() << " (" << datalog.numLines()
            << " lines)" << std::endl;

  // plot windows must be destroyed before GLFW shuts down
  tau_right_plot.reset();
  tau_left_plot.reset();
  cp_planner_plot.reset();
  foot_plot.reset();
  sim.Close();
  return 0;
}