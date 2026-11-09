#include "control/qp_tsid.h"
#include "common/urdf_geometry.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <tsid/math/utils.hpp>

namespace biped_controllers
{
    // constructor
    QP_TSID::QP_TSID(tsid::robots::RobotWrapper &robot, const std::string &yaml_config_path,
                     const std::string &urdf_path)
    : yaml_config_path_(yaml_config_path), urdf_path_(urdf_path), robot_wrapper_(robot),
      tsid_("tsid", robot, false) // TSID formulation on the caller's robot model
    {
        this->yaml_config_ = YAML::LoadFile(yaml_config_path); // read yaml config file

        // create the tasks from the yaml (not added to the TSID formulation yet)
        this->setupContactTask(); // first: it sets the foot frames used by the swing tasks
        this->setupSwingTask();
        this->setupCoMTask();
        this->setupBaseOrientationTask();
        this->setupPostureTask();
        this->setupActuationBoundsTask();

        //preallocate variables

        //solver
        this->solver_.reset(tsid::solvers::SolverHQPFactory::createNewSolver(
        tsid::solvers::SOLVER_HQP_EIQUADPROG_FAST, "qp_solver"));

        // preallocate: references, solution
        this->com_ref_ = tsid::trajectories::TrajectorySample(3); 
        this->posture_ref_ = tsid::trajectories::TrajectorySample(this->robot_wrapper_.na());
        this->base_ref_ = tsid::trajectories::TrajectorySample(12, 6); 
        this->swing_ref_[0] = tsid::trajectories::TrajectorySample(12, 6);
        this->swing_ref_[1] = tsid::trajectories::TrajectorySample(12, 6);

        // solution buffers, zero until the first solve
        this->dv_sol_ = Eigen::VectorXd::Zero(this->robot_wrapper_.nv());
        this->tau_sol_ = Eigen::VectorXd::Zero(this->robot_wrapper_.na());
        this->f_contact_ = Eigen::VectorXd::Zero(12); // 3 force_dim x 4 points
        for (auto &w : this->wrench_sol_)
            w.setZero();

        // frame ids (foot / base poses in reset, addContact, removeContact), looked up once:
        // getFrameId searches the frames by name
        this->foot_id_[0] = this->robot_wrapper_.model().getFrameId(this->left_foot_frame_);
        this->foot_id_[1] = this->robot_wrapper_.model().getFrameId(this->right_foot_frame_);
        this->base_id_ = this->base_orientation_task_->frame_id();

        // initial task stack: double support (references are set by reset() before the first solve)
        this->StackInitialWalkingTask();
    }

    void QP_TSID::reset(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        // kinematics of the current state into the formulation's data (frame poses, CoM)
        pinocchio::Data &data = this->tsid_.data();
        this->robot_wrapper_.computeAllTerms(data, q, v); // compute forward kinematics

        // per foot: contact task, swing task, force regularization weight
        tsid::contacts::Contact6d *contact[2] = {this->left_foot_contact_task.get(), this->right_foot_contact_task.get()};
        tsid::tasks::TaskSE3Equality *swing[2] = {this->left_foot_swing_task.get(), this->right_foot_swing_task.get()};
        const double force_reg_weight[2] = {this->left_contact_force_reg_weight_, this->right_contact_force_reg_weight_};

        for (int i = 0; i < 2; ++i)
        {
            // contact reference = current foot pose (Contact6d starts at the identity pose: without
            // this TSID pulls the foot to the world origin)
            const pinocchio::SE3 oMf = this->robot_wrapper_.framePosition(data, this->foot_id_[i]);
            contact[i]->setReference(oMf);

            // swing reference = current foot pose too (no jump when the foot lifts off later)
            tsid::math::SE3ToVector(oMf, this->swing_ref_[i].pos);
            this->swing_ref_[i].vel.setZero();
            this->swing_ref_[i].acc.setZero();
            swing[i]->setReference(this->swing_ref_[i]);

            // both feet in contact: a foot that was lifted gets its contact back, its swing task off
            if (!this->contact_active_[i])
            {
                this->tsid_.removeTask(swing[i]->name());
                this->tsid_.addRigidContact(*contact[i], force_reg_weight[i], 1.0, 0);
                this->contact_active_[i] = true;
            }
        }

        // CoM reference = current CoM, at rest
        this->com_ref_.pos = this->robot_wrapper_.com(data);
        this->com_ref_.vel.setZero();
        this->com_ref_.acc.setZero();
        this->com_task_->setReference(this->com_ref_);

        // base orientation reference = current base rotation, at rest (translation masked out)
        tsid::math::SE3ToVector(this->robot_wrapper_.framePosition(data, this->base_id_), this->base_ref_.pos);
        this->base_ref_.vel.setZero();
        this->base_ref_.acc.setZero();
        this->base_orientation_task_->setReference(this->base_ref_);

        // posture reference = current joint positions (q: 7 base coordinates, then the na joints)
        this->posture_ref_.pos = q.tail(this->robot_wrapper_.na());
        this->posture_ref_.vel.setZero();
        this->posture_ref_.acc.setZero();
        this->posture_task_->setReference(this->posture_ref_);
    }

    void QP_TSID::StackInitialWalkingTask()
    {
        // add left / right leg contact tasks: (contact, force regularization weight, motion weight,
        // motion priority 0 = hard constraint, the feet do not move)
        this->tsid_.addRigidContact(*this->left_foot_contact_task, this->left_contact_force_reg_weight_, 1.0, 0);
        this->tsid_.addRigidContact(*this->right_foot_contact_task, this->right_contact_force_reg_weight_, 1.0, 0);
        this->contact_active_[0] = true;
        this->contact_active_[1] = true;

        // add CoM task: balance (CoM over the feet) and height
        this->tsid_.addMotionTask(*this->com_task_, this->com_setting_.weight, this->com_setting_.priority, 0.0);

        // add base orientation task: torso upright
        this->tsid_.addMotionTask(*this->base_orientation_task_, this->base_orientation_setting_.weight,
                                  this->base_orientation_setting_.priority, 0.0);

        // add postural task - joint space tracking (low weight: only the redundant directions)
        this->tsid_.addMotionTask(*this->posture_task_, this->posture_setting_.weight, this->posture_setting_.priority, 0.0);

        // add torque limits (priority 0 by default: hard constraint)
        this->tsid_.addActuationTask(*this->actuation_bounds_task_, this->actuation_bounds_setting_.weight,
                                     this->actuation_bounds_setting_.priority, 0.0);

        // swing tasks: not added here (only while a foot is in the air)
    }

    void QP_TSID::setBaseReference(const pinocchio::SE3 &pose, const Eigen::Matrix<double, 6, 1> &vel,
                                   const Eigen::Matrix<double, 6, 1> &acc)
    {
        // written into the preallocated sample (SE3 as a 12-vector, derivatives 6D)
        tsid::math::SE3ToVector(pose, this->base_ref_.pos);
        this->base_ref_.vel = vel;
        this->base_ref_.acc = acc;
        this->base_orientation_task_->setReference(this->base_ref_);
    }

    void QP_TSID::setCoMReference(const Eigen::VectorXd &pos, const Eigen::VectorXd &vel,
                                  const Eigen::VectorXd &acc)
    {
        // written into the preallocated sample (size 3: no allocation)
        this->com_ref_.pos = pos;
        this->com_ref_.vel = vel;
        this->com_ref_.acc = acc;
        this->com_task_->setReference(this->com_ref_);
    }

    void QP_TSID::setPostureReference(const Eigen::VectorXd &qj, const Eigen::VectorXd &dqj,
                                      const Eigen::VectorXd &ddqj)
    {
        // written into the preallocated sample (same size: no allocation)
        this->posture_ref_.pos = qj;
        this->posture_ref_.vel = dqj;
        this->posture_ref_.acc = ddqj;
        this->posture_task_->setReference(this->posture_ref_);
    }

    void QP_TSID::setSwingReference(int i, const pinocchio::SE3 &pose, const Eigen::Matrix<double, 6, 1> &vel,
                                    const Eigen::Matrix<double, 6, 1> &acc)
    {
        // written into the preallocated sample (SE3 as a 12-vector, derivatives 6D)
        tsid::math::SE3ToVector(pose, this->swing_ref_[i].pos);
        this->swing_ref_[i].vel = vel;
        this->swing_ref_[i].acc = acc;
        tsid::tasks::TaskSE3Equality *swing[2] = {this->left_foot_swing_task.get(), this->right_foot_swing_task.get()};
        swing[i]->setReference(this->swing_ref_[i]);
    }

    void QP_TSID::liftFoot(int i)
    {
        if (!this->contact_active_[i])
            return;
        tsid::contacts::Contact6d *contact[2] = {this->left_foot_contact_task.get(), this->right_foot_contact_task.get()};
        tsid::tasks::TaskSE3Equality *swing[2] = {this->left_foot_swing_task.get(), this->right_foot_swing_task.get()};

        // contact off (force ramped down by TSID), swing task on
        this->tsid_.removeRigidContact(contact[i]->name(), this->contact_transition_time_);
        this->tsid_.addMotionTask(*swing[i], this->swing_setting_.weight, this->swing_setting_.priority, 0.0);
        this->contact_active_[i] = false;
    }

    void QP_TSID::landFoot(int i)
    {
        // input: i - foot index, 0 = left, 1 = right
        if (this->contact_active_[i])
            return;
        tsid::contacts::Contact6d *contact[2] = {this->left_foot_contact_task.get(), this->right_foot_contact_task.get()};
        tsid::tasks::TaskSE3Equality *swing[2] = {this->left_foot_swing_task.get(), this->right_foot_swing_task.get()};
        const double force_reg_weight[2] = {this->left_contact_force_reg_weight_, this->right_contact_force_reg_weight_};

        // swing task off, contact on where the foot is (state of the last solve), not where it was
        // planned: no pull toward the planned foothold
        this->tsid_.removeTask(swing[i]->name(), 0.0);
        // TSID ramped the max normal force to ~0 at liftoff and does not restore it: without this
        // the landed foot carries (almost) no load
        contact[i]->setMaxNormalForce(this->contact_f_max_[i]);
        contact[i]->setReference(this->robot_wrapper_.framePosition(this->tsid_.data(), this->foot_id_[i]));
        this->tsid_.addRigidContact(*contact[i], force_reg_weight[i], 1.0, 0);
        this->contact_active_[i] = true;
    }

    pinocchio::SE3 QP_TSID::getFootPose(int i)
    {
        // same source as the contact reference of landFoot: TSID data of the last solve
        return this->robot_wrapper_.framePosition(this->tsid_.data(), this->foot_id_[i]);
    }

    bool QP_TSID::solve(double t, const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        // QP of this cycle from the current tasks, contacts and state
        const tsid::solvers::HQPData &data = this->tsid_.computeProblemData(t, q, v);

        // the problem size changes when a contact is added or removed: size the solver every call
        this->solver_->resize(this->tsid_.nVar(), this->tsid_.nEq(), this->tsid_.nIn());

        const tsid::solvers::HQPOutput &sol = this->solver_->solve(data);
        if (sol.status != tsid::solvers::HQP_STATUS_OPTIMAL)
            return false;

        // joint accelerations and torques
        this->dv_sol_ = this->tsid_.getAccelerations(sol);
        this->tau_sol_ = this->tsid_.getActuatorForces(sol);

        // contact wrenches: the 12 corner forces -> 6D wrench (force generator matrix)
        tsid::contacts::Contact6d *contact[2] = {this->left_foot_contact_task.get(), this->right_foot_contact_task.get()};
        for (int i = 0; i < 2; ++i)
        {
            if (this->contact_active_[i] && this->tsid_.getContactForces(contact[i]->name(), sol, this->f_contact_))
                this->wrench_sol_[i].noalias() = contact[i]->getForceGeneratorMatrix() * this->f_contact_;
            else
                this->wrench_sol_[i].setZero();
        }
        return true;
    }

    YAML::Node QP_TSID::contactConfig(const std::string &side) const
    {
        // the contact_tasks entry of this foot: its contact_frame contains "left" / "right"
        for (const auto &entry : this->yaml_config_["contact_tasks"])
        {
            if (entry["contact_frame"].as<std::string>().find(side) != std::string::npos)
                return entry;
        }
        throw std::runtime_error("[QP_TSID] no contact_tasks entry for the " + side + " foot in '" +
                                 this->yaml_config_path_ + "'");
    }

    void QP_TSID::setupContactTask()
    {
        // one Contact6d per foot (left_foot_contact_task, right_foot_contact_task), same settings
        // layout: a lambda builds one from its yaml entry
        auto make_contact = [this](const std::string &side, std::string &frame, double &force_reg_weight)
        {
            const YAML::Node cfg = this->contactConfig(side);

            // contact frame: must exist in the model
            frame = cfg["contact_frame"].as<std::string>();
            if (!this->robot_wrapper_.model().existFrame(frame))
                throw std::runtime_error("[QP_TSID] contact frame '" + frame + "' is not in the URDF");

            // sole corners (in the contact frame) from the foot's collision box in the URDF;
            // Contact6d needs exactly 4 points
            const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
            Eigen::Matrix3Xd corners;
            if (!utils::soleCornersFromUrdf(this->urdf_path_, frame, normal, corners) || corners.cols() != 4)
                throw std::runtime_error("[QP_TSID] no 4-corner sole for '" + frame + "' in the URDF");

            // friction cone and normal force bounds
            const double mu = cfg["mu"].as<double>();
            const double f_min = cfg["f_min"].as<double>();
            const double f_max = cfg["f_max"].as<double>();
            auto contact = std::make_shared<tsid::contacts::Contact6d>(
                "contact_" + side, this->robot_wrapper_, frame, corners, normal, mu, f_min, f_max);

            // contact acceleration gains (kd defaults to 2*sqrt(kp), critical damping)
            const double kp = cfg["kp"].as<double>();
            const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
            contact->Kp(kp * Eigen::VectorXd::Ones(6));
            contact->Kd(kd * Eigen::VectorXd::Ones(6));

            // force regularization weight, used by addRigidContact
            force_reg_weight = cfg["weight"] ? cfg["weight"].as<double>() : 1e-3;
            return contact;
        };

        this->left_foot_contact_task = make_contact("left", this->left_foot_frame_, this->left_contact_force_reg_weight_);
        this->right_foot_contact_task = make_contact("right", this->right_foot_frame_, this->right_contact_force_reg_weight_);
        this->contact_f_max_[0] = this->left_foot_contact_task->getMaxNormalForce();
        this->contact_f_max_[1] = this->right_foot_contact_task->getMaxNormalForce();
    }

    void QP_TSID::setupSwingTask()
    {
        // one SE3 task per foot (same frames as the contacts, so setupContactTask runs first),
        // same gains for both, from the swing_foot_task section
        const YAML::Node cfg = this->yaml_config_["swing_foot_task"];
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->swing_setting_.weight = cfg["weight"].as<double>();
        this->swing_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
        this->contact_transition_time_ = cfg["contact_transition_time"] ? cfg["contact_transition_time"].as<double>() : 0.15;

        auto make_swing = [this, kp, kd](const std::string &side, const std::string &frame)
        {
            auto task = std::make_shared<tsid::tasks::TaskSE3Equality>("swing_" + side, this->robot_wrapper_, frame);
            task->Kp(kp * Eigen::VectorXd::Ones(6));
            task->Kd(kd * Eigen::VectorXd::Ones(6));
            // local frame (TSID default), as tsid_examples: the error is taken in the foot frame; the
            // world-frame reference twist is mapped into it by TSID
            return task;
        };

        this->left_foot_swing_task = make_swing("left", this->left_foot_frame_);
        this->right_foot_swing_task = make_swing("right", this->right_foot_frame_);
    }

    void QP_TSID::setupCoMTask()
    {
        const YAML::Node cfg = this->yaml_config_["com_task"];

        // gains (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->com_task_ = std::make_shared<tsid::tasks::TaskComEquality>("com", this->robot_wrapper_);
        this->com_task_->Kp(kp * Eigen::Vector3d::Ones());
        this->com_task_->Kd(kd * Eigen::Vector3d::Ones());

        // controlled axes [x, y, z]
        if (cfg["mask"])
        {
            const auto mask = cfg["mask"].as<std::vector<double>>();
            if (mask.size() != 3)
                throw std::runtime_error("[QP_TSID] com_task/mask must have 3 values");
            this->com_task_->setMask(Eigen::Map<const Eigen::Vector3d>(mask.data()));
        }

        this->com_setting_.weight = cfg["weight"].as<double>();
        this->com_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupBaseOrientationTask()
    {
        const YAML::Node cfg = this->yaml_config_["base_orientation_task"];

        // base frame: must exist in the model
        const std::string frame = cfg["frame"].as<std::string>();
        if (!this->robot_wrapper_.model().existFrame(frame))
            throw std::runtime_error("[QP_TSID] base frame '" + frame + "' is not in the URDF");

        // gains (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->base_orientation_task_ = std::make_shared<tsid::tasks::TaskSE3Equality>("base_orientation", this->robot_wrapper_, frame);
        this->base_orientation_task_->Kp(kp * Eigen::VectorXd::Ones(6));
        this->base_orientation_task_->Kd(kd * Eigen::VectorXd::Ones(6));
        // error and reference in the world frame (mask z = world vertical, e.g. the base height)
        this->base_orientation_task_->useLocalFrame(false);

        // controlled axes [x, y, z, roll, pitch, yaw]: by default orientation only
        Eigen::VectorXd mask(6);
        mask << 0, 0, 0, 1, 1, 1; // just control roll, pitch, yaw
        if (cfg["mask"])
        {
            const auto m = cfg["mask"].as<std::vector<double>>();
            if (m.size() != 6)
                throw std::runtime_error("[QP_TSID] base_orientation_task/mask must have 6 values");
            mask = Eigen::Map<const Eigen::VectorXd>(m.data(), 6);
        }
        this->base_orientation_task_->setMask(mask);

        this->base_orientation_setting_.weight = cfg["weight"].as<double>();
        this->base_orientation_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupPostureTask()
    {
        const YAML::Node cfg = this->yaml_config_["posture_task"];
        const int na = this->robot_wrapper_.na();

        // default gains on every actuated joint (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"] ? cfg["kp"].as<double>() : 10.0;
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        Eigen::VectorXd Kp = kp * Eigen::VectorXd::Ones(na);
        Eigen::VectorXd Kd = kd * Eigen::VectorXd::Ones(na);

        // optional per-joint gains by joint name (joint_gain: {name: {kp, kd}}), in Pinocchio order
        // (the joint's velocity index minus the 6 base velocities)
        const pinocchio::Model &model = this->robot_wrapper_.model();
        for (const auto &entry : cfg["joint_gain"])
        {
            const std::string name = entry.first.as<std::string>();
            if (!model.existJointName(name))
                throw std::runtime_error("[QP_TSID] posture_task/joint_gain: joint '" + name + "' is not in the URDF");
            const int j = model.joints[model.getJointId(name)].idx_v() - 6;
            Kp(j) = entry.second["kp"].as<double>();
            Kd(j) = entry.second["kd"] ? entry.second["kd"].as<double>() : 2.0 * std::sqrt(Kp(j));
        }

        // critical damping (kd = 2*sqrt(kp)) on every joint unless use_critical_damping is false: it
        // overrides the kd above, as the earlier task parser of tsid_examples
        if (!cfg["use_critical_damping"] || cfg["use_critical_damping"].as<bool>())
        {
            Kd = 2.0 * Kp.cwiseSqrt();
        }

        this->posture_task_ = std::make_shared<tsid::tasks::TaskJointPosture>("posture", this->robot_wrapper_);
        this->posture_task_->Kp(Kp);
        this->posture_task_->Kd(Kd);

        this->posture_setting_.weight = cfg["weight"].as<double>();
        this->posture_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupActuationBoundsTask()
    {
        const YAML::Node cfg = this->yaml_config_["actuation_bounds_task"];
        const int na = this->robot_wrapper_.na();

        // torque limits -tau_max <= tau <= tau_max, in the model's joint order (Pinocchio order).
        // Default: the URDF <limit effort="..."> of each joint (effortLimit is nv long, the
        // actuated joints are its last na entries)
        Eigen::VectorXd upper = this->robot_wrapper_.model().effortLimit.tail(na);

        // optional override from the yaml (e.g. while the URDF still has placeholder limits)
        if (cfg["tau_max"])
        {
            const auto tau_max = cfg["tau_max"].as<std::vector<double>>();
            if (static_cast<int>(tau_max.size()) != na)
                throw std::runtime_error("[QP_TSID] actuation_bounds_task/tau_max has " + std::to_string(tau_max.size()) +
                                         " values, the robot has " + std::to_string(na) + " joints");
            upper = Eigen::Map<const Eigen::VectorXd>(tau_max.data(), na);
        }
        this->actuation_bounds_task_ = std::make_shared<tsid::tasks::TaskActuationBounds>("actuation_bounds", this->robot_wrapper_);
        this->actuation_bounds_task_->setBounds(-upper, upper);

        this->actuation_bounds_setting_.weight = cfg["weight"] ? cfg["weight"].as<double>() : 1.0;
        this->actuation_bounds_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 0;
    }
} // namespace biped_controllers