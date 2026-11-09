#include "control/wbc_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <yaml-cpp/yaml.h>

#include "common/urdf_geometry.h"
#include "common/utils.h"   // computeIK_Leg, quinticTrajectory

namespace biped_controllers
{
    namespace
    {
        // log line with a level tag, as RCLCPP_INFO / WARN / ERROR in the ROS 2 project
        void logLine(FILE *stream, const char *level, const char *format, va_list args)
        {
            std::fprintf(stream, "[WBC]%s ", level);
            std::vfprintf(stream, format, args);
            std::fprintf(stream, "\n");
        }
        void logInfo(const char *format, ...)
        {
            va_list args;
            va_start(args, format);
            logLine(stdout, "", format, args);
            va_end(args);
        }
        void logWarn(const char *format, ...)
        {
            va_list args;
            va_start(args, format);
            logLine(stderr, "[warn]", format, args);
            va_end(args);
        }
        void logError(const char *format, ...)
        {
            va_list args;
            va_start(args, format);
            logLine(stderr, "[error]", format, args);
            va_end(args);
        }
    }

    // ---------------------------------------------------------------------
    // on_configure of the ROS 2 controller: robot model, modules, per-cycle buffers, drive gains
    // ---------------------------------------------------------------------
    WBCController::WBCController(const ControllerParams &params) : params_(params)
    {
        this->setupRobotModel();
        this->setupWbc();
        this->setupStateEstimate();
        this->AllocateBuffers();
        this->loadDriveGains();
    }

    // ---------------------------------------------------------------------
    // on_activate of the ROS 2 controller: estimators, TSID and references from the current state
    // ---------------------------------------------------------------------
    void WBCController::activate(const RobotSensor &sensor, const GroundTruth *ground_truth)
    {
        this->sensor_ = sensor;
        this->ground_truth_ = ground_truth;

        // estimators, TSID and planner start from the current state
        this->resetModules();

        // targets of commands 1 / 2 (IK once), references held at the current state
        this->setupPostureTargets();
        this->command_pending_ = false;
        logInfo("Active: holding the current pose");
    }

    bool WBCController::update(double time, const RobotSensor &sensor, const GroundTruth *ground_truth)
    {
        // all values finite (a broken sensor reads NaN): keep the last command
        const auto &joints = sensor.actuator_state;
        const auto &imu = sensor.imu_sensor;
        if (!joints.qj.allFinite() || !joints.dqj.allFinite() || !joints.torquej.allFinite() ||
            !imu.imu_quat_.coeffs().allFinite() || !imu.imu_gyro_L.allFinite() || !imu.imu_accel_L.allFinite())
        {
            logError("invalid sensor reading (NaN / Inf): keeping the last command");
            return false;
        }
        this->sensor_ = sensor;
        this->ground_truth_ = ground_truth;

        // estimate base state and contact state
        this->estimate();

        // motion command: may start a transition from the state just estimated
        this->handleMotionCommand(time);

        // compute the joint command (planner + TSID)
        return this->computeCommand(time);
    }

    void WBCController::sendCommand(MotionCommand command)
    {
        this->command_ = command;
        this->command_pending_ = true;
    }

    void WBCController::walk(int num_steps)
    {
        this->walk_steps_ = std::max(num_steps, 0);
        this->sendCommand(MotionCommand::WALKING);
    }

    void WBCController::setTurnCommand(double wz)
    {
        this->turn_cmd_ = std::clamp(wz, -1.0, 1.0);
    }

    // ---------------------------------------------------------------------
    // constructor: loads the URDF as a free-flyer robot model, the one model every module uses
    // ---------------------------------------------------------------------
    void WBCController::setupRobotModel ()
    {
        // no package directories: the robot model needs no meshes
        const std::vector<std::string> package_dirs;
        this->robot_pin_wrapper_ = std::make_unique<tsid::robots::RobotWrapper>(
            this->params_.urdf_path, package_dirs, pinocchio::JointModelFreeFlyer(), false);
        const tsid::robots::RobotWrapper &robot = this->robotModel();
        logInfo("Robot model: nq %d, nv %d, na %d", robot.nq(), robot.nv(), robot.na());
    }

    // ---------------------------------------------------------------------
    // constructor: QP_TSID (TSID tasks and contacts) on the robot model and the LIP walk planner,
    // both from the config yaml; control period dt_ of all modules
    // ---------------------------------------------------------------------
    void WBCController::setupWbc ()
    {
        // QP_TSID on the shared robot model; the URDF for the sole geometry of the contacts
        this->qp_tsid_controller_ = std::make_unique<QP_TSID>(this->robotModel(), this->params_.config_path,
                                                              this->params_.urdf_path);

        // walk planner: section "LIPPlanner" of the same yaml, support frames on the foot frames
        this->walk_planner_ = std::make_unique<WalkPlanner>(this->robotModel(), this->params_.config_path,
                                                            this->params_.left_foot_frame,
                                                            this->params_.right_foot_frame);

        // control period of all modules: sampling_frequency of the yaml
        this->dt_ = this->walk_planner_->getDt();
    }

    // ---------------------------------------------------------------------
    // constructor, after setupWbc (needs dt_): state estimator and contact fusion on the robot model
    // ---------------------------------------------------------------------
    void WBCController::setupStateEstimate ()
    {
        const tsid::robots::RobotWrapper &robot = this->robotModel();
        const std::string feet[2] = {this->params_.left_foot_frame, this->params_.right_foot_frame};

        // state estimator: base position / velocity from IMU + leg kinematics
        this->state_estimator_ = std::make_unique<StateEstimator>(robot, this->dt_, feet[0], feet[1]);

        // contact fusion: foot contact probability from the momentum observer, the foot height
        // and the gait phase (observer cutoff 15 Hz)
        this->contact_fusion_ = std::make_unique<ContactFusion>(
            robot.model(), std::vector<std::string>{feet[0], feet[1]}, this->dt_, 15.0);

        // sole corners of each foot: bottom face of the foot's collision box in the URDF (the same
        // source as the TSID contacts), for the foot height of the contact fusion
        for (std::size_t i = 0; i < 2; ++i)
        {
            Eigen::Matrix3Xd corners;
            if (utils::soleCornersFromUrdf(this->params_.urdf_path, feet[i], Eigen::Vector3d::UnitZ(), corners))
            {
                this->contact_fusion_->setSoleCorners(i, corners);
            }
            else
            {
                logWarn("No sole box in the URDF for foot frame '%s': the contact fusion uses the "
                        "frame origin as the sole", feet[i].c_str());
            }
        }

        logInfo("Control modules ready: QP_TSID, WalkPlanner, StateEstimator, ContactFusion (dt %.4f s)", this->dt_);
    }

    // ---------------------------------------------------------------------
    // activate(): start the estimators from the current state (sensor_ holds a first reading)
    // ---------------------------------------------------------------------
    void WBCController::resetModules ()
    {
        // state estimator: standing on both feet, base above the origin at the height given by the
        // leg kinematics, feet on the ground
        this->state_estimator_->reset(this->sensor_);

        // contact fusion: both feet in contact, observer restarted
        this->contact_fusion_->reset();

        // contact probability: both feet (same as the contact fusion after reset); used by the
        // state estimator in the first update()
        this->contact_probability_.setOnes();

        // TSID: every reference at the estimated state, both feet in contact where they are now
        this->state_estimator_->getPinocchioState(this->q_, this->v_);
        if (this->params_.use_ground_truth)
        {
            this->applyGroundTruth();
        }
        this->qp_tsid_controller_->reset(this->q_, this->v_);
    }

    // ---------------------------------------------------------------------
    // update(): base state and contact state of this cycle
    // ---------------------------------------------------------------------
    void WBCController::estimate ()
    {
        // base state: IMU + leg kinematics of the legs in contact. Contact = the contact in effect
        // (scheduled_contact_ of the last cycle, 1 stance / 0 swing: the planner's phase, and with
        // use_time_base false the swing foot from its detected touchdown on), not the fused contact
        // probability below (it lags the liftoff: the lifting leg's kinematics would still be trusted)
        const Eigen::Vector2d planned_contact(this->scheduled_contact_[0] ? 1.0 : 0.0,
                                              this->scheduled_contact_[1] ? 1.0 : 0.0);
        this->state_estimator_->update(this->sensor_, planned_contact);
        this->state_estimator_->getPinocchioState(this->q_, this->v_);
        if (this->params_.use_ground_truth)
        {
            this->applyGroundTruth();
        }

        // contact state: momentum observer with the measured joint torques -> foot forces, then
        // fused with the foot height and the planned contact -> probability for the next cycle
        this->contact_fusion_->computeDisturbanceTorque(this->q_, this->v_, this->sensor_.actuator_state.torquej);
        this->contact_probability_ =
            this->contact_fusion_->fuseContactProbability(this->scheduled_contact_, this->contact_phase_);

    }

    // ---------------------------------------------------------------------
    // Base state from the ground truth. MuJoCo free joint: position and linear velocity in the world
    // frame, angular velocity in the body frame; Pinocchio wants both velocities in the base frame
    // ---------------------------------------------------------------------
    void WBCController::applyGroundTruth ()
    {
        if (!this->ground_truth_)
        {
            logWarn("use_ground_truth: no ground truth given, using the estimate");
            return;
        }
        const GroundTruth &gt = *this->ground_truth_;

        // pose: position, quaternion (x y z w)
        this->q_.head<3>() = gt.position;
        this->q_.segment<4>(3) = gt.orientation.coeffs();

        // twist in the base frame: linear rotated from the world frame, angular as is
        this->v_.head<3>().noalias() = gt.orientation.toRotationMatrix().transpose() * gt.linear_velocity;
        this->v_.segment<3>(3) = gt.angular_velocity;
    }

    // ---------------------------------------------------------------------
    // update(): joint command from the estimated state (Pinocchio order).
    // Returns false while there is no command yet: the last one is held.
    // ---------------------------------------------------------------------
    bool WBCController::computeCommand (double time)
    {
        // set task based on joystick command, then process the task:
        //   command 1: homing, postural task only, control all the joints to the home position (B6)
        //   command 2: init walking, lower the CoM to the reference zc (bend the knees) (B8)
        //   command 3: walking with constant step length and swing time (B10)
        //   command 4: heading turning (B11)
        //   command 5: transition from walking to stop (B11)
        // each fills q_cmd_ / v_cmd_ / tau_cmd_ from the TSID solution (dv_, tau_) and returns true

        // walking (command 3): planner references
        if (this->mode_ == Mode::WALK)
        {
            return this->computeWalkCommand(time);
        }

        // commands 1 / 2, a transition from the last reference to the target of the
        // command (handleMotionCommand); after it (or before any command) the target is held
        const double t = time - this->transition_start_time_;

        // joint reference of this cycle: quintic to the joint target
        utils::quinticTrajectory(this->qj_start_, this->qj_target_, t, kTransitionDuration, this->qj_ref_,
                                 this->dqj_ref_, this->ddqj_ref_);
        this->qp_tsid_controller_->setPostureReference(this->qj_ref_, this->dqj_ref_, this->ddqj_ref_);

        // CoM reference of this cycle: same timing, to the CoM of the target pose
        utils::quinticTrajectory(this->com_start_, this->com_target_, t, kTransitionDuration, this->com_ref_,
                                 this->dcom_ref_, this->ddcom_ref_);
        this->qp_tsid_controller_->setCoMReference(this->com_ref_, this->dcom_ref_, this->ddcom_ref_);

        // TSID: accelerations and torques for it with both feet in contact; failed -> hold
        if (!this->qp_tsid_controller_->solve(time, this->q_, this->v_))
        {
            logError("TSID QP failed (t %.3f s): keeping the last command", time);
            return false;
        }
        this->dv_ = this->qp_tsid_controller_->getAccelerations();
        this->tau_ = this->qp_tsid_controller_->getTorques();

        // drive command: the motor PD tracks the joint reference, the TSID torque is the feedforward
        this->q_cmd_ = this->qj_ref_;
        this->v_cmd_ = this->dqj_ref_;
        this->tau_cmd_ = this->tau_;
        return true;
    }

    // ---------------------------------------------------------------------
    // activate(): targets of commands 1 / 2 and every reference at the current state.
    //   command 1 (homing): all joints 0
    //   command 2 (init walking): IK once, both feet keep their pose relative to the base, only
    //   lower (base kStandBaseHeight above the ground)
    // The CoM of each target pose is kept in the left foot frame: startTransition places it on the
    // foot where it is then (the feet do not move in these commands).
    // ---------------------------------------------------------------------
    void WBCController::setupPostureTargets ()
    {
        const pinocchio::Model &model = this->robotModel().model();
        pinocchio::Data data(model);
        const int na = this->robotModel().na();

        // base and foot poses of the estimated state (q_ from resetModules)
        pinocchio::framesForwardKinematics(model, data, this->q_);
        const pinocchio::SE3 oMb = data.oMi[1]; // free-flyer joint = base (copy: the IK below reuses data)
        const pinocchio::SE3 oMf[2] = {data.oMf[model.getFrameId(this->params_.left_foot_frame)],
                                       data.oMf[model.getFrameId(this->params_.right_foot_frame)]};

        // command 2: IK target relative to the base, same foot x, y and orientation; foot z relative
        // to the base = foot z in the world - desired base height (ground at z = 0 in the estimator)
        pinocchio::SE3 bMf[2];
        for (int i = 0; i < 2; ++i)
        {
            bMf[i] = oMb.actInv(oMf[i]);
            bMf[i].translation().z() = oMf[i].translation().z() - kStandBaseHeight;
        }
        const utils::IkRes ik = utils::computeIK_Leg(model, data, bMf[0].rotation(), bMf[0].translation(),
                                                     bMf[1].rotation(), bMf[1].translation());
        if (ik.status != 0)
        {
            throw std::runtime_error("init walking: IK for the base height " + std::to_string(kStandBaseHeight) +
                                     " m did not converge");
        }
        this->qj_stand_ = ik.jointPosRes;

        // command 1: all joints 0
        this->qj_home_ = Eigen::VectorXd::Zero(na);

        // CoM of each target pose in the left foot frame (base pose does not matter: identity)
        Eigen::VectorXd q_target = pinocchio::neutral(model);
        const Eigen::VectorXd *qj_targets[2] = {&this->qj_home_, &this->qj_stand_};
        Eigen::Vector3d *com_in_foot[2] = {&this->com_home_in_foot_, &this->com_stand_in_foot_};
        for (int k = 0; k < 2; ++k)
        {
            q_target.tail(na) = *qj_targets[k];
            const Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q_target);
            pinocchio::framesForwardKinematics(model, data, q_target);
            *com_in_foot[k] = data.oMf[this->left_foot_id_].actInv(com);
        }

        // every reference at the current state, held until the first command (start = target)
        this->qj_start_ = this->q_.tail(na);
        this->qj_target_ = this->qj_start_;
        this->qj_ref_ = this->qj_start_;
        this->dqj_ref_ = Eigen::VectorXd::Zero(na);
        this->ddqj_ref_ = Eigen::VectorXd::Zero(na);
        this->com_start_ = pinocchio::centerOfMass(model, data, this->q_);
        this->com_target_ = this->com_start_;
        this->com_ref_ = this->com_start_;
        this->dcom_ref_ = Eigen::VectorXd::Zero(3);
        this->ddcom_ref_ = Eigen::VectorXd::Zero(3);
        this->transition_start_time_ = -std::numeric_limits<double>::infinity();
        this->mode_ = Mode::STAND;
        this->in_walking_pose_ = false;

        logInfo("motion targets: base now %.3f m; homing: joints 0; init walking: base %.3f m "
                    "(knee %.3f / %.3f rad); both over %.1f s", oMb.translation().z(), kStandBaseHeight,
                    this->qj_stand_(3), this->qj_stand_(9), kTransitionDuration);
    }

    // ---------------------------------------------------------------------
    // constructor: drive gains of the joints, from the gains yaml. Section `robot`: the ankle gains
    // are those of the two ankle motors, each ankle joint takes their sum, as the simulated drive
    // (an approximation of the linkage). A file without that section: the joint gains as they are.
    // ---------------------------------------------------------------------
    void WBCController::loadDriveGains ()
    {
        const pinocchio::Model &model = this->robotModel().model();
        const int na = this->robotModel().na();
        this->drive_kp_ = Eigen::VectorXd::Zero(na);
        this->drive_kd_ = Eigen::VectorXd::Zero(na);

        YAML::Node file;
        try
        {
            file = YAML::LoadFile(this->params_.gains_path);
        }
        catch (const std::exception &e)
        {
            logWarn("gains file: %s; drive gains 0", e.what());
            return;
        }
        const bool parallel_ankles = static_cast<bool>(file["robot"]);
        const YAML::Node gains = parallel_ankles ? file["robot"] : file;

        // kp / kd of each joint (actuated index: velocity index after the 6 free-flyer DoF)
        int found = 0;
        for (const auto &entry : gains)
        {
            const std::string name = entry.first.as<std::string>();
            if (!model.existJointName(name))
            {
                continue;
            }
            const int j = model.joints[model.getJointId(name)].idx_v() - 6;
            this->drive_kp_(j) = entry.second["kp"].as<double>();
            this->drive_kd_(j) = entry.second["kd"].as<double>();
            ++found;
        }
        if (parallel_ankles)
        {
            for (const char *side : {"left", "right"})
            {
                const std::string roll = std::string(side) + "_ankle_roll_joint";
                const std::string pitch = std::string(side) + "_ankle_pitch_joint";
                if (!model.existJointName(roll) || !model.existJointName(pitch))
                {
                    continue;
                }
                const int r = model.joints[model.getJointId(roll)].idx_v() - 6;
                const int p = model.joints[model.getJointId(pitch)].idx_v() - 6;
                const double kp = this->drive_kp_(r) + this->drive_kp_(p);
                const double kd = this->drive_kd_(r) + this->drive_kd_(p);
                this->drive_kp_(r) = this->drive_kp_(p) = kp;
                this->drive_kd_(r) = this->drive_kd_(p) = kd;
            }
        }
        if (found != na)
        {
            logWarn("drive gains found for %d of %d joints (the others 0)", found, na);
        }
    }

    // ---------------------------------------------------------------------
    // update(): starts the transition of a new motion command.
    // ---------------------------------------------------------------------
    void WBCController::handleMotionCommand (double time)
    {
        // nothing new since the last cycle
        if (!this->command_pending_)
        {
            return;
        }
        this->command_pending_ = false;
        const bool transition_running = this->transitionRunning(time);

        // walking: the first command at rest starts it
        if (this->command_ == MotionCommand::WALKING)
        {
            if (this->mode_ == Mode::WALK)
            {
                return;
            }
            if (transition_running || !this->in_walking_pose_)
            {
                logWarn("walking ignored: %s", transition_running ? "a transition is running"
                                                                  : "send init walking first");
                return;
            }
            this->startWalking(time);
            return;
        }
        if (this->mode_ == Mode::WALK)
        {
            logWarn("motion command %d ignored while walking", static_cast<int>(this->command_));
            return;
        }

        // one transition at a time: the next one starts from a target, at rest
        if (transition_running)
        {
            logWarn("motion command %d ignored: a transition is running", static_cast<int>(this->command_));
            return;
        }

        // target of the command
        if (this->command_ == MotionCommand::HOMING)
        {
            this->startTransition(this->qj_home_, this->com_home_in_foot_, time);
            this->in_walking_pose_ = false;
            logInfo("command 1: homing (joints to 0) over %.1f s", kTransitionDuration);
        }
        else
        {
            this->startTransition(this->qj_stand_, this->com_stand_in_foot_, time);
            this->in_walking_pose_ = true;
            logInfo("command 2: init walking (base to %.3f m) over %.1f s", kStandBaseHeight, kTransitionDuration);
        }
    }

    // ---------------------------------------------------------------------
    // update(): new transition from the references of this cycle (no jump in the command) to the
    // joint target and the CoM of that pose on the left foot.
    // ---------------------------------------------------------------------
    void WBCController::startTransition (const Eigen::VectorXd &qj_target, const Eigen::Vector3d &com_in_foot,
                                         double time)
    {
        const pinocchio::Model &model = this->robotModel().model();

        // left foot pose of the estimated state (FK on the preallocated data_)
        pinocchio::forwardKinematics(model, this->data_, this->q_);
        const pinocchio::SE3 &oMf = pinocchio::updateFramePlacement(model, this->data_, this->left_foot_id_);

        // start at the current references, end at the target
        this->qj_start_ = this->qj_ref_;
        this->qj_target_ = qj_target;
        this->com_start_ = this->com_ref_;
        this->com_target_ = oMf.act(com_in_foot);
        this->transition_start_time_ = time;
    }

    // ---------------------------------------------------------------------
    // handleMotionCommand: start walking from the current state (standing at rest, init walking
    // pose, both feet in contact).
    // ---------------------------------------------------------------------
    void WBCController::startWalking (double /*time*/)
    {
        // planner from the estimated state: support frame on the left ankle, LIP at the CoM height
        this->walk_planner_->reset(this->q_);
        this->walk_planner_->setWaitForTouchdown(!this->params_.use_time_base); // event based: late steps wait
        this->prev_swing_foot_ = -1; // touchdown detection: no step yet
        this->liftoff_seen_ = false;
        this->swing_state_ = SwingState::SWING;

        // base upright at the yaw between the feet (translation masked out in the task); turning
        // moves it with the feet (computeWalkCommand)
        this->base_pose_.rotation() =
            Eigen::AngleAxisd(this->walk_planner_->getRefBaseYaw(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        this->qp_tsid_controller_->setBaseReference(this->base_pose_, this->zero6_, this->zero6_);

        // joint command: from the last one until the first walk cycle (the posture reference stays
        // the stand pose)
        this->q_des_ = this->q_cmd_;
        this->v_des_ = this->v_cmd_;

        this->mode_ = Mode::WALK;
        this->stop_requested_ = false;
        this->stop_command_ = false;
        logInfo("command 3: walking at %.3f m/s (CoM height %.3f m)",
                    this->params_.walk_vx, this->walk_planner_->getRefCoMPosition().z());
    }

    // ---------------------------------------------------------------------
    // computeCommand while walking.
    // ---------------------------------------------------------------------
    bool WBCController::computeWalkCommand (double time)
    {
        const int na = this->robotModel().na();

        // the last full step is under way (or stopWalking()): finish it, closing step, stop and rest
        const bool last_step = this->walk_steps_ > 0 && this->walk_planner_->getWalkPhase() == WalkPhase::STEP &&
                               this->walk_planner_->getStepCount() >= this->walk_steps_ - 1;
        if (!this->stop_requested_ && (this->stop_command_ || last_step))
        {
            this->walk_planner_->requestStop();
            this->stop_requested_ = true;
            logInfo("stopping: step %d is the last full step", this->walk_planner_->getStepCount() + 1);
        }

        // turn command, times the largest yaw rate
        const double turn_cmd = this->turn_cmd_;
        this->walk_planner_->setYawRateCommand(turn_cmd * this->params_.max_yaw_rate);

        // planner speed: the walking speed of the yaml (walk_vx, 0 = stepping in place); turning
        // happens in place (no step length while a turn is commanded). Both from the next step on
        this->walk_planner_->setVelocityCommand(turn_cmd != 0.0 ? 0.0 : this->params_.walk_vx);
        this->walk_planner_->planWalking(this->q_, this->v_);
        const bool swinging = this->walk_planner_->isSwinging();
        const int swing_foot = this->walk_planner_->getSwingFoot();

        // contact FSM of the swing foot (use_time_base false, Bledt et al. ICRA 2018, Sec. VI) with
        // the contact estimator (estimate() of this cycle); an early contact counts only after the
        // foot was seen off the ground in this step and from kMinTouchdownPhase on. A confirmed
        // touchdown (CONTACT) ends the planner's step at once (touchdown()), early or late, and its
        // double support starts; without it the step waits at Tswing (late)
        if (swinging && swing_foot != this->prev_swing_foot_) // first cycle of a new step (other foot swings)
        {
            this->liftoff_seen_ = false;                       // the new swing foot is still down
            this->swing_state_ = SwingState::SWING;            // FSM restarts: follow the swing trajectory
        }
        // run the FSM only in event mode, during a step, until the touchdown is confirmed
        if (!this->params_.use_time_base && swinging && this->swing_state_ != SwingState::CONTACT)
        {
            const bool contact = this->contact_fusion_->getContactState(swing_foot); // estimated contact s_hat
            const double phase = this->walk_planner_->getPhase();                   // step phase in [0, 1)
            const char *foot_name = swing_foot == 0 ? "left" : "right";             // for the logs
            this->liftoff_seen_ = this->liftoff_seen_ || !contact;                   // latched once the foot is seen in the air
            if (this->swing_state_ == SwingState::SWING) // SWING: waiting for a touchdown
            {
                // SWING -> EARLY: contact before the step ends, stop the trajectory and hold the foot
                // where it touched down (flat, the planned orientation); no load on it yet
                // (liftoff seen: not the contact still reported right after liftoff; phase: no touchdown
                // in the first half of the swing)
                if (this->liftoff_seen_ && contact && phase >= kMinTouchdownPhase)
                {
                    this->swing_state_ = SwingState::EARLY;                // stop following the trajectory
                    this->early_start_time_ = time;              // t0: start of the confirmation timer
                    this->hold_pose_.translation() = this->qp_tsid_controller_->getFootPose(swing_foot).translation(); // hold where it touched down
                    this->hold_pose_.rotation() = this->walk_planner_->getRefSwingFootRotation();                   // flat, planned yaw
                    logInfo("early touchdown of the %s foot at step phase %.2f: holding the foot",
                                foot_name, phase);
                }
                // SWING -> LATE: the step reached its end (planner waits at phase 1) without contact
                else if (this->walk_planner_->isLate())
                {
                    this->swing_state_ = SwingState::LATE; // the planner's swing reference descends below the foothold
                    logInfo("late touchdown of the %s foot: no contact at the end of the step, descending",
                                foot_name);
                }
            }
            else if (this->swing_state_ == SwingState::LATE) // LATE: waiting for any contact
            {
                const double late_time = this->walk_planner_->getLateTime(); // [s] past Tswing
                if (contact) // LATE -> CONTACT: the paper's s_hat = 1, no confirmation (the foot is overdue)
                {
                    this->swing_state_ = SwingState::CONTACT;
                    logInfo("late touchdown of the %s foot %.3f s after the step end: contact on",
                                foot_name, late_time);
                }
                else if (this->params_.late_touchdown_timeout > 0.0 && late_time >= this->params_.late_touchdown_timeout)
                {
                    this->swing_state_ = SwingState::CONTACT; // timeout: the foot lands in TSID in the air
                    logWarn("no touchdown of the %s foot %.3f s after the step end: contact on anyway",
                                foot_name, late_time);
                }
            }
            else if (!contact) // EARLY, contact gone
            {
                // EARLY -> SWING: fleeting contact (lost before the confirmation), the trajectory resumes
                this->swing_state_ = SwingState::SWING; // back to the swing reference of the planner
                logInfo("contact of the %s foot lost after %.3f s: swing resumes", foot_name,
                            time - this->early_start_time_);
            }
            else if (time - this->early_start_time_ >= this->params_.touchdown_confirm_time) // EARLY, contact held t >= t0 + t_delay
            {
                // EARLY -> CONTACT: the contact held for t_delay, the foot carries load (lands below)
                this->swing_state_ = SwingState::CONTACT; // landFoot in the contact loop below; FSM idle until the next step
                logInfo("touchdown of the %s foot confirmed at step phase %.2f: contact on",
                            foot_name, phase);
            }
            // touchdown: the planner ends the step at Tswing, or in its next cycle if it is late
            if (this->swing_state_ == SwingState::CONTACT)
            {
                this->walk_planner_->touchdown();
            }
        }
        this->prev_swing_foot_ = swinging ? swing_foot : -1;                          // detects the next step
        const bool in_swing = swinging && this->swing_state_ != SwingState::CONTACT; // SWING / EARLY / LATE: swing task, no contact

        // contacts: the plan, the swing foot down once landed; land first, then lift with its
        // swing reference set. Contact fusion prior (next estimate()): the plan and the swing phase,
        // 0.5 with both feet down (double support, as tsid_examples); a LATE swing foot is scheduled
        // in contact (paper Sec. VI, s_phi = 1), its stance phase counting from the step end (the
        // prior favors a contact more as the foot gets later)
        for (int i = 0; i < 2; ++i) // i = 0 left, 1 right foot
        {
            // in contact unless it is the foot currently swinging (SWING / EARLY / LATE)
            const bool in_contact = !(in_swing && swing_foot == i);
            // landed foot (CONTACT, or the stance foot): add its TSID contact if not active yet
            if (in_contact && !this->qp_tsid_controller_->isContactActive(i))
            {
                this->qp_tsid_controller_->landFoot(i);
            }
            // LATE swing foot: still swinging in TSID, but the fusion prior expects its contact
            const bool late_foot = !in_contact && this->swing_state_ == SwingState::LATE;
            this->scheduled_contact_[i] = in_contact || late_foot; // s_phi of the contact fusion
            // phase prior: LATE = time past the step end / Tswing, walking = swing phase, else 0.5
            this->contact_phase_[i] = late_foot  ? this->walk_planner_->getLateTime() / this->walk_planner_->getTswing()
                                      : swinging ? this->walk_planner_->getPhase()
                                                 : 0.5;
        }
        if (in_swing)
        {
            // swing trajectory of the planner, or (EARLY) the touchdown pose at rest
            if (this->swing_state_ == SwingState::EARLY)
            {
                this->swing_pose_ = this->hold_pose_;
                this->swing_vel_.setZero();
                this->swing_acc_.setZero();
            }
            else
            {
                this->swing_pose_.rotation() = this->walk_planner_->getRefSwingFootRotation();
                this->swing_pose_.translation() = this->walk_planner_->getRefSwingFootPosition();
                this->swing_vel_.head<3>() = this->walk_planner_->getRefSwingFootVelocity();
                this->swing_vel_.tail<3>() << 0.0, 0.0, this->walk_planner_->getRefSwingFootYawRate();
                this->swing_acc_.head<3>() = this->walk_planner_->getRefSwingFootAcceleration();
                this->swing_acc_.tail<3>() << 0.0, 0.0, this->walk_planner_->getRefSwingFootYawAcc();
            }
            this->qp_tsid_controller_->setSwingReference(swing_foot, this->swing_pose_, this->swing_vel_, this->swing_acc_);
            if (this->qp_tsid_controller_->isContactActive(swing_foot))
            {
                this->qp_tsid_controller_->liftFoot(swing_foot);
            }
        }

        // torso yaw between the feet (upright), with its rate / acceleration feedforward
        this->base_pose_.rotation() =
            Eigen::AngleAxisd(this->walk_planner_->getRefBaseYaw(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        this->base_vel_(5) = this->walk_planner_->getRefBaseYawRate();
        this->base_acc_(5) = this->walk_planner_->getRefBaseYawAcc();
        this->qp_tsid_controller_->setBaseReference(this->base_pose_, this->base_vel_, this->base_acc_);

        // CoM reference of the planner (copied into the size-3 buffers: no temporary)
        this->com_ref_ = this->walk_planner_->getRefCoMPosition();
        this->dcom_ref_ = this->walk_planner_->getRefCoMVelocity();
        this->ddcom_ref_ = this->walk_planner_->getRefCoMAcceleration();
        this->qp_tsid_controller_->setCoMReference(this->com_ref_, this->dcom_ref_, this->ddcom_ref_);

        // QP solver
        // TSID with the contacts of the plan; failed -> hold
        if (!this->qp_tsid_controller_->solve(time, this->q_, this->v_))
        {
            logError("TSID QP failed while walking (t %.3f s): keeping the last command", time);
            return false;
        }

        // Output acceleration and torque cmd
        this->dv_ = this->qp_tsid_controller_->getAccelerations();
        this->tau_ = this->qp_tsid_controller_->getTorques();

        // drive command (PVT): the measured joint state integrated one cycle with the TSID
        // accelerations, v_des = v + dv dt, q_des = q + v dt + dv dt^2 / 2 (from the measurement each
        // cycle: no drift), and the feedforward torque minus the drive PD at that state,
        //   tau_ff = tau - [kp (q_des - q) + kd (v_des - v)]
        // so the drive applies the TSID torque at the measured state, and its PD only acts on the
        // deviation from the predicted motion (latency, between the cycles). torque_mode: only the
        // effort is sent (drive kp = kd = 0), tau_ff = tau
        const double dt = this->dt_;
        const auto qj = this->q_.tail(na), vj = this->v_.tail(na), dvj = this->dv_.tail(na);
        this->v_des_ = vj + dt * dvj;
        this->q_des_ = qj + dt * vj + (0.5 * dt * dt) * dvj;
        this->q_cmd_ = this->q_des_;
        this->v_cmd_ = this->v_des_;
        if (this->params_.torque_mode)
        {
            this->tau_cmd_ = this->tau_;
        }
        else
        {
            this->tau_cmd_ = this->tau_ - (this->drive_kp_.cwiseProduct(this->q_des_ - qj) +
                                           this->drive_kd_.cwiseProduct(this->v_des_ - vj));
        }

        // at rest (double support) once the planned CoM has reached its CP (the middle of the feet,
        // it converges after the stop phase): standing again, every reference held where it is (the
        // next init walking / homing starts from here)
        const Eigen::Vector2d com_to_cp = this->walk_planner_->getRefCP() - this->com_ref_.head<2>();
        if (this->walk_planner_->getWalkPhase() == WalkPhase::REST && com_to_cp.norm() < kRestTolerance)
        {
            this->mode_ = Mode::STAND;
            this->qj_ref_ = this->q_des_;
            this->qj_start_ = this->q_des_;
            this->qj_target_ = this->q_des_;
            this->com_start_ = this->com_ref_;
            this->com_target_ = this->com_ref_;
            this->transition_start_time_ = -std::numeric_limits<double>::infinity();
            this->v_des_.setZero();
            this->scheduled_contact_.assign(2, true);
            logInfo("walking stopped: standing at rest");
        }
        return true;
    }

    // ---------------------------------------------------------------------
    // Sizes the per-cycle buffers once (constructor, after setupRobotModel).
    // ---------------------------------------------------------------------
    void WBCController::AllocateBuffers ()
    {
        const tsid::robots::RobotWrapper &robot = this->robotModel();
        const int nq = robot.nq(); // 19: base position 3 + quaternion 4 + 12 joints
        const int nv = robot.nv(); // 18: base twist 6 + 12 joints
        const int na = robot.na(); // 12 actuated joints

        // sensor data of one cycle (joint vectors of size na inside, IMU fixed-size)
        this->sensor_ = RobotSensor(na);

        // estimated state in Pinocchio / TSID convention
        this->q_ = Eigen::VectorXd::Zero(nq);
        this->v_ = Eigen::VectorXd::Zero(nv);

        // contact probability per foot [left, right]; reset to [1, 1] in on_activate
        this->contact_probability_ = Eigen::VectorXd::Ones(2);

        // planned contact (contact fusion input): standing, both feet in stance mid-phase
        this->scheduled_contact_.assign(2, true);
        this->contact_phase_.assign(2, 0.5);

        // TSID solution
        this->dv_ = Eigen::VectorXd::Zero(nv);
        this->tau_ = Eigen::VectorXd::Zero(na);

        // joint command, Pinocchio order (sent by the caller)
        this->q_cmd_ = Eigen::VectorXd::Zero(na);
        this->v_cmd_ = Eigen::VectorXd::Zero(na);
        this->tau_cmd_ = Eigen::VectorXd::Zero(na);

        // walking: integrated joint command, swing foot reference
        this->q_des_ = Eigen::VectorXd::Zero(na);
        this->v_des_ = Eigen::VectorXd::Zero(na);
        this->swing_pose_ = pinocchio::SE3::Identity();
        this->swing_vel_.setZero();
        this->swing_acc_.setZero();

        // kinematics of a motion command start (startTransition)
        this->data_ = pinocchio::Data(robot.model());
        this->left_foot_id_ = robot.model().getFrameId(this->params_.left_foot_frame);
    }

} // namespace biped_controllers
