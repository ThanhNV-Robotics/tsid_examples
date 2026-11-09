// Biped walking with the whole-body controller (control/wbc_controller.h, port of the ROS 2 project):
// the robot lowers to the walking pose, walks a given number of steps, closes with a step that brings
// the feet side by side, stops, stands for 2 s and closes.
//
//   ./build/biped_walking             num_steps of config/controllers.yaml (section walk_demo)
//   ./build/biped_walking 8           8 full steps, then the closing step
//   ./build/biped_walking 0           walk until the window is closed
//
// Options: config/controllers.yaml (controller, demo), config/tsid_wbc.yaml (tasks, LIP planner),
// config/joint_gains.yaml (drive gains, torque_mode false).
// Environment: TSID_HEADLESS=1 hidden windows; TSID_SIM_TIME=<s> ends the run at that simulated time.
//
// As lockstep:=true of the ROS 2 simulation: the controller runs every control period, the physics
// exactly period / timestep steps in between, with the drive law applied at every physics step.

// Standard library
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Third party
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Project
#include "common/data_logger.h"               // CSV logging
#include "common/data_type.h"                 // RobotSensor, RobotState
#include "common/utils.h"
#include "control/wbc_controller.h"
#include "estimation/cheat_state_estimator.h" // simulator contacts and ground truth
#include "sim/Robot_Simulator.h"              // MuJoCo simulator, viewer and RealtimePlot

using namespace std;
using namespace Eigen;
using namespace biped_controllers;

namespace {

// Scripted run: the commands of the keyboard in the ROS 2 project
enum class DemoState { SETTLE, INIT_WALKING, WAIT_WALK, WALKING, STOPPED };

// roll, pitch, yaw (ZYX) of a rotation
Vector3d rpyOf(const Matrix3d &R) {
  return Vector3d(std::atan2(R(2, 1), R(2, 2)), std::asin(-R(2, 0)), std::atan2(R(1, 0), R(0, 0)));
}

// the base state of the MuJoCo free joint (first joint of the model)
GroundTruth groundTruth(const mjData *d) {
  GroundTruth gt;
  gt.position = Map<const Vector3d>(d->qpos);
  gt.orientation = Quaterniond(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
  gt.linear_velocity = Map<const Vector3d>(d->qvel);
  gt.angular_velocity = Map<const Vector3d>(d->qvel + 3);
  return gt;
}

} // namespace

int main(int argc, char **argv) {
  const string ROOT = PROJECT_ROOT_DIR;
  const string XML_PATH = ROOT + "/models/biped/mjcf/scene_wbc_12dof_v2.xml";
  const string CONTROLLERS_PATH = ROOT + "/config/controllers.yaml";

  // ------------------------------------------------------------------------
  // 1. Options: controller and demo from the yaml, number of steps from the command line
  // ------------------------------------------------------------------------
  ControllerParams params;
  params.urdf_path = ROOT + "/models/biped/urdf/v2_biped_robot_12dof.urdf";
  params.config_path = ROOT + "/config/tsid_wbc.yaml";
  params.gains_path = ROOT + "/config/joint_gains.yaml";
  params.load(CONTROLLERS_PATH);

  const YAML::Node demo_cfg = YAML::LoadFile(CONTROLLERS_PATH)["walk_demo"];
  int num_steps = demo_cfg["num_steps"] ? demo_cfg["num_steps"].as<int>() : 4;
  const double turn_command = demo_cfg["turn_command"] ? demo_cfg["turn_command"].as<double>() : 0.0;
  const double settle_time = demo_cfg["settle_time"] ? demo_cfg["settle_time"].as<double>() : 0.5;
  const double camera_distance = demo_cfg["camera_distance"] ? demo_cfg["camera_distance"].as<double>() : 3.0;
  const double camera_azimuth = demo_cfg["camera_azimuth"] ? demo_cfg["camera_azimuth"].as<double>() : 150.0;
  const double camera_elevation = demo_cfg["camera_elevation"] ? demo_cfg["camera_elevation"].as<double>() : -20.0;
  const bool camera_track = demo_cfg["camera_track"] && demo_cfg["camera_track"].as<bool>();
  const bool record_video = demo_cfg["record_video"] && demo_cfg["record_video"].as<bool>();
  const double video_fps = demo_cfg["video_fps"] ? demo_cfg["video_fps"].as<double>() : 30.0;
  const int video_width = demo_cfg["video_width"] ? demo_cfg["video_width"].as<int>() : 1920;
  const int video_height = demo_cfg["video_height"] ? demo_cfg["video_height"].as<int>() : 1080;
  const int video_crf = demo_cfg["video_crf"] ? demo_cfg["video_crf"].as<int>() : 18;
  if (argc > 1) {
    char *end = nullptr;
    const long steps = std::strtol(argv[1], &end, 10);
    if (*end != '\0' || steps < 0) {
      std::fprintf(stderr, "usage: %s [num_steps >= 0]\n", argv[0]);
      return 1;
    }
    num_steps = static_cast<int>(steps);
  }

  std::printf("====================================================\n");
  std::printf("  Biped walking: WBC + LIP planner with MuJoCo      \n");
  std::printf("====================================================\n");
  params.print();
  if (num_steps > 0) {
    std::printf("[Demo] %d full step(s), then the closing step and stop\n", num_steps);
  } else {
    std::printf("[Demo] walking until the window is closed\n");
  }

  // ------------------------------------------------------------------------
  // 2. Simulator, from the "home" keyframe (feet flat on the floor)
  // ------------------------------------------------------------------------
  Robot_Simulator sim(XML_PATH);
  const string video_path = ROOT + "/record/biped_walking.mp4";
  sim.setVideoFile(video_path, video_fps, video_width, video_height, video_crf);
  sim.init("Biped WBC walking", record_video); // creates the viewer window
  sim.setCamera(camera_distance, camera_azimuth, camera_elevation, camera_track);
  const int home = mj_name2id(sim.model(), mjOBJ_KEY, "home");
  if (home < 0) {
    std::fprintf(stderr, "no keyframe 'home' in %s\n", XML_PATH.c_str());
    return 1;
  }
  sim.setInitConfiguration(Map<const VectorXd>(sim.model()->key_qpos + home * sim.model()->nq, sim.model()->nq));

  // ------------------------------------------------------------------------
  // 3. Controller
  // ------------------------------------------------------------------------
  WBCController wbc(params);
  tsid::robots::RobotWrapper &robot = wbc.robotModel();
  const int na = robot.na();

  // RobotSensor comes in actuator order, the controller works in Pinocchio order: they must match
  const vector<string> mj_joints = utils::getJointNames(sim.model());
  for (int i = 0; i < na; ++i) {
    const string &name = mj_joints[mj_joints.size() - na + i];
    if (robot.model().names[2 + i] != name) { // 0 = universe, 1 = free flyer
      std::fprintf(stderr, "joint %d: '%s' in MuJoCo, '%s' in Pinocchio\n", i, name.c_str(),
                   robot.model().names[2 + i].c_str());
      return 1;
    }
  }

  // Sim step per control cycle, and the simulator's view of the contacts (logs, plots)
  const int physics_steps = std::max(1, static_cast<int>(std::lround(wbc.getDt() / sim.model()->opt.timestep)));
  std::printf("[Demo] control %.0f Hz, %d physics step(s) of %.4f s per cycle\n", 1.0 / wbc.getDt(), physics_steps,
              sim.model()->opt.timestep);
  Cheat_StateEstimator cheat(sim.model(), sim.data());

  RobotSensor sensor = sim.getRobotSensorValues();
  GroundTruth gt = groundTruth(sim.data());
  wbc.activate(sensor, &gt);
  wbc.setTurnCommand(turn_command);

  // Drive of the joints (biped_pvt_drive in the ROS 2 project), PVT or pure torque
  const VectorXd kp = params.torque_mode ? VectorXd(VectorXd::Zero(na)) : wbc.getDriveKp();
  const VectorXd kd = params.torque_mode ? VectorXd(VectorXd::Zero(na)) : wbc.getDriveKd();
  VectorXd qj(na), dqj(na), tau = VectorXd::Zero(na);

  // ------------------------------------------------------------------------
  // 4. Log (read by scripts/eval_walking.py, plot_data.py)
  // ------------------------------------------------------------------------
  const pinocchio::FrameIndex lf_id = robot.model().getFrameId(params.left_foot_frame);
  const pinocchio::FrameIndex rf_id = robot.model().getFrameId(params.right_foot_frame);
  pinocchio::Data pin_data(robot.model()); // measured feet and CoM

  DataLogger datalog(ROOT + "/record/biped_walking.csv");
  const vector<string> leg_joint_names(mj_joints.end() - na, mj_joints.end());
  const vector<string> xy{"x", "y"}, xyz{"x", "y", "z"}, lr{"L", "R"}, rpy{"roll", "pitch", "yaw"};
  datalog.addItem("time", 1);
  datalog.addItem("pos_cmd", leg_joint_names);
  datalog.addItem("pos_measured", leg_joint_names);
  datalog.addItem("tau_cmd", leg_joint_names);
  datalog.addItem("com_ref", xyz);
  datalog.addItem("com_meas", xyz);
  datalog.addItem("base_rpy", rpy);
  datalog.addItem("imu_accel", xyz); // [m/s^2] torso IMU, sensor frame, gravity included (+9.81 on z at rest)
  datalog.addItem("swing_leg", 1); // 1 = left foot swings, -1 = right foot swings, 0 = both down
  datalog.addItem("swing_phase", 1);
  datalog.addItem("swing_state", 1); // contact FSM: 0 swing, 1 early, 2 late, 3 contact
  datalog.addItem("late_touchdown", 1);
  datalog.addItem("double_support", 1);
  datalog.addItem("step_count", 1);
  datalog.addItem("plan_com", xy);
  datalog.addItem("plan_zmp", xy);
  datalog.addItem("plan_cp", xy);
  datalog.addItem("meas_cp", xy);
  datalog.addItem("swing_land", xyz);
  datalog.addItem("swing_ref", xyz);
  datalog.addItem("swing_ref_vel", xyz);
  datalog.addItem("lfoot_meas", xyz);
  datalog.addItem("rfoot_meas", xyz);
  datalog.addItem("lfoot_rpy", rpy);
  datalog.addItem("rfoot_rpy", rpy);
  datalog.addItem("f_est_z", lr);
  datalog.addItem("f_gt_z", lr);
  datalog.addItem("sole_z", lr);
  datalog.addItem("ground_z", lr);
  datalog.addItem("p_phase", lr);
  datalog.addItem("p_height", lr);
  datalog.addItem("p_force", lr);
  datalog.addItem("p_fused", lr);
  datalog.addItem("s_hat", lr);
  datalog.addItem("s_gt", lr);
  datalog.finishItemAdding();

  // Real-time plots
  auto force_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450, "Foot contact force", 10.0);
  force_plot->setYLabel("Fz (N)");
  force_plot->setLineWidth(2.0);
  auto contact_state_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                           "Contact state: estimated vs ground truth", 10.0);
  contact_state_plot->setYLabel("contact (L: 0/1, R: 1.5/2.5)");
  contact_state_plot->setLineWidth(2.0);

  // ------------------------------------------------------------------------
  // 5. Simulation & control loop
  // ------------------------------------------------------------------------
  std::cout << "[SIM] Press '1' to pause/resume, 'T' to turn camera tracking on/off; Drag robot body to test compliance.\n";

  const char *sim_time_env = std::getenv("TSID_SIM_TIME");
  const double max_sim_time = sim_time_env ? std::atof(sim_time_env) : std::numeric_limits<double>::infinity();

  const double stop_hold_time = 2.0; // [s] standing after the stop, then the run ends

  DemoState demo_state = DemoState::SETTLE;
  double state_start = sim.time(); // [s] start of the current demo state
  double max_tilt = 0.0;           // [rad] largest torso tilt from the first step on
  bool walked = false;             // the planner has run (its references are valid)
  bool failed = false;             // QP failed / the robot fell
  bool done = false;               // the run is over
  long tick = 0;                   // physics steps

  while (!sim.shouldClose() && !failed && !done && sim.time() <= max_sim_time) {
    const double simstart = sim.time();

    // Step physics at ~60 Hz visual frame rate
    while (sim.time() - simstart < 1.0 / 60.0 && sim.runSim && !failed && !done) {
      const double t = sim.time();

      // 5.1 Control cycle
      if (tick % physics_steps == 0) {
        sensor = sim.getRobotSensorValues();
        const RobotState state = cheat.estimate(sensor);
        gt = groundTruth(sim.data());

        // The script: the keyboard commands of the ROS 2 project
        switch (demo_state) {
        case DemoState::SETTLE:
          if (t - state_start >= settle_time) {
            wbc.sendCommand(MotionCommand::INIT_WALKING);
            demo_state = DemoState::INIT_WALKING;
            state_start = t;
          }
          break;
        case DemoState::INIT_WALKING:
          // the command starts in the next update(): wait one cycle before checking the transition
          if (t - state_start > 2.0 * wbc.getDt() && !wbc.transitionRunning(t)) {
            demo_state = DemoState::WAIT_WALK;
            state_start = t;
          }
          break;
        case DemoState::WAIT_WALK:
          if (t - state_start >= settle_time) {
            wbc.walk(num_steps);
            demo_state = DemoState::WALKING;
            state_start = t;
          }
          break;
        case DemoState::WALKING:
          walked = walked || wbc.isWalking();
          if (walked && !wbc.isWalking()) {
            // feet of the last log line: side by side after the closing step
            const Vector3d lf = pin_data.oMf[lf_id].translation(), rf = pin_data.oMf[rf_id].translation();
            std::printf("[Demo] t=%.3f stopped after %d step(s), closing step included; left foot (%.3f, %.3f), "
                        "right foot (%.3f, %.3f) m\n", t, wbc.getWalkPlanner().getStepCount(), lf.x(), lf.y(), rf.x(), rf.y());
            demo_state = DemoState::STOPPED;
            state_start = t;
          }
          break;
        case DemoState::STOPPED:
          done = t - state_start >= stop_hold_time;
          break;
        }

        if (!wbc.update(t, sensor, &gt)) {
          std::printf("[Demo] no command at t=%.3f (QP failed / invalid sensor), stopping\n", t);
          failed = true;
          break;
        }

        // Fall check: the torso far from upright, or too low
        const Vector3d rpy_base = rpyOf(state.quat_b_W.toRotationMatrix());
        const double tilt = std::acos(std::clamp(state.quat_b_W.toRotationMatrix()(2, 2), -1.0, 1.0));
        if (walked) max_tilt = std::max(max_tilt, tilt);
        if (tilt > 0.8 || state.pos_b_W.z() < 0.4) {
          std::printf("[Demo] the robot fell at t=%.3f (tilt %.1f deg, base height %.3f m), stopping\n", t,
                      tilt * 180.0 / M_PI, state.pos_b_W.z());
          failed = true;
          break;
        }

        // 5.2 Log: measured feet and CoM from the ground truth
        VectorXd q_gt(robot.nq());
        q_gt << gt.position, gt.orientation.coeffs(), sensor.actuator_state.qj;
        pinocchio::forwardKinematics(robot.model(), pin_data, q_gt);
        pinocchio::updateFramePlacements(robot.model(), pin_data);
        const Vector3d com_meas = pinocchio::centerOfMass(robot.model(), pin_data, q_gt);

        const WalkPlanner &planner = wbc.getWalkPlanner();
        const ContactFusion &fusion = wbc.getContactFusion();
        const bool swinging = walked && planner.isSwinging();
        datalog.startNewLine();
        datalog.recItemData("time", t);
        datalog.recItemData("pos_cmd", wbc.getJointPosCmd());
        datalog.recItemData("pos_measured", sensor.actuator_state.qj);
        datalog.recItemData("tau_cmd", wbc.getTorqueCmd());
        datalog.recItemData("com_ref", wbc.getComRef());
        datalog.recItemData("com_meas", com_meas);
        datalog.recItemData("base_rpy", rpy_base);
        datalog.recItemData("imu_accel", sensor.imu_sensor.imu_accel_L);
        datalog.recItemData("swing_leg", !swinging ? 0.0 : planner.getSwingFoot() == 0 ? 1.0 : -1.0);
        datalog.recItemData("swing_phase", walked ? planner.getPhase() : 0.0);
        datalog.recItemData("swing_state", static_cast<double>(wbc.getSwingState()));
        datalog.recItemData("late_touchdown", walked && planner.isLate() ? 1.0 : 0.0);
        datalog.recItemData("double_support", walked && planner.inDoubleSupport() ? 1.0 : 0.0);
        datalog.recItemData("step_count", static_cast<double>(planner.getStepCount()));
        datalog.recItemData("plan_com", Vector2d(planner.getRefCoMPosition().head<2>()));
        datalog.recItemData("plan_zmp", planner.getRefZMP());
        datalog.recItemData("plan_cp", planner.getRefCP());
        datalog.recItemData("meas_cp", planner.getMeasuredCP());
        datalog.recItemData("swing_land", planner.getFoothold());
        datalog.recItemData("swing_ref", planner.getRefSwingFootPosition());
        datalog.recItemData("swing_ref_vel", planner.getRefSwingFootVelocity());
        datalog.recItemData("lfoot_meas", pin_data.oMf[lf_id].translation());
        datalog.recItemData("rfoot_meas", pin_data.oMf[rf_id].translation());
        datalog.recItemData("lfoot_rpy", rpyOf(pin_data.oMf[lf_id].rotation()));
        datalog.recItemData("rfoot_rpy", rpyOf(pin_data.oMf[rf_id].rotation()));
        datalog.recItemData("f_est_z", Vector2d(fusion.getFootWrench(0)(2), fusion.getFootWrench(1)(2)));
        datalog.recItemData("f_gt_z", Vector2d(cheat.getContactForces()[0].z(), cheat.getContactForces()[1].z()));
        datalog.recItemData("sole_z", Vector2d(fusion.getFootHeight(0), fusion.getFootHeight(1)));
        datalog.recItemData("ground_z", Vector2d(fusion.getGroundHeight(0), fusion.getGroundHeight(1)));
        datalog.recItemData("p_phase", Vector2d(fusion.getPhaseProbability(0), fusion.getPhaseProbability(1)));
        datalog.recItemData("p_height", Vector2d(fusion.getHeightProbability(0), fusion.getHeightProbability(1)));
        datalog.recItemData("p_force", Vector2d(fusion.getForceProbability(0), fusion.getForceProbability(1)));
        datalog.recItemData("p_fused", Vector2d(fusion.getContactProbability(0), fusion.getContactProbability(1)));
        datalog.recItemData("s_hat", Vector2d(fusion.getContactState(0), fusion.getContactState(1)));
        datalog.recItemData("s_gt", Vector2d(state.contact_flags[0], state.contact_flags[1]));
        datalog.finishLine();
      }

      // 5.3 Drive law at every physics step, then physics (and interactive perturbations)
      sim.getActuatedState(qj, dqj, na);
      tau = wbc.getTorqueCmd() + kp.cwiseProduct(wbc.getJointPosCmd() - qj) + kd.cwiseProduct(wbc.getJointVelCmd() - dqj);
      sim.setControl(tau);
      sim.stepPhysics();
      ++tick;
    }

    // Estimated vs. ground-truth (MuJoCo contacts) vertical force of each foot
    const ContactFusion &fusion = wbc.getContactFusion();
    force_plot->addPoint("L_est", sim.time(), fusion.getFootWrench(0)(2));
    force_plot->addPoint("L_gt", sim.time(), cheat.getContactForces()[0].z());
    force_plot->addPoint("R_est", sim.time(), fusion.getFootWrench(1)(2));
    force_plot->addPoint("R_gt", sim.time(), cheat.getContactForces()[1].z());
    force_plot->render();

    // Estimated contact state vs. the simulator's; right foot shifted by 1.5 so the feet do not overlap
    const std::vector<bool> &contact_gt = cheat.getState().contact_flags;
    if (contact_gt.size() >= 2) {
      contact_state_plot->addPoint("s_hat_L", sim.time(), fusion.getContactState(0) ? 1.0 : 0.0);
      contact_state_plot->addPoint("gt_L", sim.time(), contact_gt[0] ? 1.0 : 0.0);
      contact_state_plot->addPoint("s_hat_R", sim.time(), fusion.getContactState(1) ? 2.5 : 1.5);
      contact_state_plot->addPoint("gt_R", sim.time(), contact_gt[1] ? 2.5 : 1.5);
      contact_state_plot->render();
    }

    // // Yellow: measured CoM. Red: planned ZMP on the floor, with a line from the CoM
    // const float yellow[4] = {1.0f, 0.85f, 0.1f, 1.0f};
    // const float red[4] = {1.0f, 0.15f, 0.15f, 1.0f};
    // const Vector3d com_sim = sim.getCoM();
    // sim.addSphere(com_sim, 0.025, yellow);
    // if (wbc.isWalking()) {
    //   const Vector2d zmp = wbc.getWalkPlanner().getRefZMP();
    //   const Vector3d zmp_des(zmp.x(), zmp.y(), 0.0);
    //   sim.addLine(com_sim, zmp_des, red, 2.0);
    //   sim.addSphere(zmp_des, 0.015, red);
    // }
    sim.updateScene();
  }

  // ------------------------------------------------------------------------
  // 6. Result
  // ------------------------------------------------------------------------
  datalog.close();
  const Vector3d base(sim.data()->qpos[0], sim.data()->qpos[1], sim.data()->qpos[2]);
  std::printf("[Demo] %s at t=%.3f: base (%.3f, %.3f, %.3f) m, largest tilt while walking %.1f deg\n",
              failed ? "FAILED" : "finished", sim.time(), base.x(), base.y(), base.z(), max_tilt * 180.0 / M_PI);
  std::cout << "Data saved to " << datalog.path() << " (" << datalog.numLines() << " lines)" << std::endl;
  if (record_video) std::cout << "Video saved to " << video_path << std::endl;

  // plot windows must be destroyed before GLFW shuts down
  force_plot.reset();
  contact_state_plot.reset();
  sim.Close();
  return failed ? 1 : 0;
}
