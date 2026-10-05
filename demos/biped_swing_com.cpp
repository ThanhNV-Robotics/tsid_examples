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
  const double hip_width = 0.27;
  const double init_base_height = 0.78;
  const double dt = 0.001;
  const double Tswing = 1.0;

  const double simulation_time = 20.0;
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
  datalog.finishItemAdding();

  // Real-time plots
  auto joystick_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                      "JoyStick Cmd", 10.0);
  joystick_plot->setYLabel("Command_JoyStick");
  joystick_plot->setLineWidth(2.0);

  auto gait_scheduler_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                      "Gait Phase", 10.0);
  gait_scheduler_plot->setYLabel("Phase Variable");
  gait_scheduler_plot->setLineWidth(2.0);

  auto cp_planner_plot = std::make_unique<RealtimePlot>(sim.model(), 900, 450,
                                                      "Capture Point Planner", 10.0);
  cp_planner_plot->setYLabel("CoM X/Y");
  cp_planner_plot->setLineWidth(2.0);

  VectorXd tau = VectorXd::Zero(robot.na());

  // ------------------------------------------------------------------------
  // 6. Simulation & Control Loop
  // ------------------------------------------------------------------------
  RobotSensor robot_sensors;
  bool startWalking = false;
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

      // ------------------------------------------------------------------------
      // Initial standing task
      // ------------------------------------------------------------------------
      if (t <= T_stand) {
        joystick.setIniPos(state.pos_b_W[0], state.pos_b_W[1],
                           state.pos_b_W[2]);
        joystick.setVxDesLPara(0, 0.1);
        joystick.setWzDesLPara(0, 0.1);
        joystick.setPzRef(state.pos_b_W[2], 0.1);
      }

      if (t >= T_stand && !startWalking)
      {
        startWalking = true;
        joystick.setMotionState(MotionState::WALK);
        gait_scheduler.start(joystick);
        cp_planner.setInitCom(robot.com(data));   
      }

      // ------------------------------------------------------------------------
      // swinging CoM task
      // ------------------------------------------------------------------------
      if (startWalking)
      {
        gait_scheduler.step();   
        cp_planner.planWarmingUp(gait_scheduler);
        
        // set tsid CoM reference from CP planner
        com_sample.setValue(cp_planner.getCoMref());
        com_task->setReference(com_sample);
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
    joystick_plot->addPoint("Vx", sim.time(), joystick.vx_W);
    joystick_plot->addPoint("Base Z", sim.time(), joystick.pz_W);
    joystick_plot->render();

    gait_scheduler_plot->addPoint("Phi", sim.time(), gait_scheduler.phi);
    gait_scheduler_plot->render();

    cp_planner_plot->addPoint("ZMP_Y", sim.time(), cp_planner.py_d_);
    cp_planner_plot->addPoint("CP_Y", sim.time(), cp_planner.cxi_y_);
    cp_planner_plot->addPoint("CoM_Y", sim.time(), cp_planner.yc_);
    cp_planner_plot->render();

    sim.updateScene();


    
  }

  // Flush and close log file
  datalog.close();
  std::cout << "Data saved to " << datalog.path() << " (" << datalog.numLines()
            << " lines)" << std::endl;
  sim.Close();
  return 0;
}