// This is the main class to solve the Task Space Inverse Dynamic problem based on Quadratic Programming

#pragma once
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

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace biped_controllers
{
    class QP_TSID
    {
    public:
        // Constructors
        // robot: free-flyer robot model, owned by the caller and shared with the other modules;
        // it must outlive this object (TSID keeps a reference to it)
        // urdf_path: the URDF of that model, read for the sole geometry of the contacts
        QP_TSID(tsid::robots::RobotWrapper &robot, const std::string &yaml_config_path,
                const std::string &urdf_path);

        // set up tasks function from config yaml
        void setupContactTask();
        void setupSwingTask();
        void setupCoMTask();
        void setupBaseOrientationTask();
        void setupPostureTask();
        void setupActuationBoundsTask();
        
        // reset/initialize
        // set all the reference at the current state
        // lifecycle: on_activate()
        void reset(const Eigen::VectorXd &q, const Eigen::VectorXd &v);

        // Stack Initial-Walking Task
        // add both contacts + CoM, base orientation, posture, torque limits (double support);
        // (CoM reference: setCoMReference, reset() sets it to the current CoM)
        // called once by the constructor
        void StackInitialWalkingTask();

        // base reference (world frame): pose, twist and acceleration [linear; angular]; only the
        // axes of the base_orientation_task mask are tracked
        void setBaseReference(const pinocchio::SE3 &pose, const Eigen::Matrix<double, 6, 1> &vel,
                              const Eigen::Matrix<double, 6, 1> &acc);

        // CoM reference (world frame): position, velocity, acceleration (3 each)
        void setCoMReference(const Eigen::VectorXd &pos, const Eigen::VectorXd &vel,
                             const Eigen::VectorXd &acc);

        // posture reference: joint positions, velocities, accelerations (na each, Pinocchio order)
        void setPostureReference(const Eigen::VectorXd &qj, const Eigen::VectorXd &dqj,
                                 const Eigen::VectorXd &ddqj);

        // swing foot reference of foot i (0 left, 1 right), world frame: ankle pose, twist and
        // acceleration [linear; angular]; tracked while the foot is lifted (liftFoot)
        void setSwingReference(int i, const pinocchio::SE3 &pose, const Eigen::Matrix<double, 6, 1> &vel,
                               const Eigen::Matrix<double, 6, 1> &acc);

        // contact switching (planner timing): liftFoot removes the contact of foot i (its force
        // ramps to zero over contact_transition_time) and adds its swing task (set the swing
        // reference first); landFoot removes the swing task and adds the contact back at the
        // foot's current pose. Not real-time: TSID allocates when tasks are added / removed
        void liftFoot(int i);
        void landFoot(int i);
        bool isContactActive(int i) const { return this->contact_active_[i]; }
        // ankle pose of foot i (0 left, 1 right), world frame, from the state of the last solve
        // (e.g. to hold a swing foot where it touched down)
        pinocchio::SE3 getFootPose(int i);

        // solve the QP of this cycle (t [s]: drives the contact transitions); false if the QP failed
        // (the last solution is kept)
        bool solve(double t, const Eigen::VectorXd &q, const Eigen::VectorXd &v);
        const Eigen::VectorXd &getAccelerations() const { return this->dv_sol_; }   // nv
        const Eigen::VectorXd &getTorques() const { return this->tau_sol_; }        // na, Pinocchio order
        // contact wrench [force; torque] of foot i (0 left, 1 right), world frame; zero in swing
        const Eigen::Matrix<double, 6, 1> &getContactWrench(int i) const { return this->wrench_sol_[i]; }

    private:
        // weight / priority of a task, read from the yaml, used when the task is added to TSID
        struct TaskSetting
        {
            double weight = 0.0;
            unsigned int priority = 1; // 0 = hard constraint, 1 = soft (weighted) task
        };

        // yaml section of the contact on the given foot ("left" / "right"): the contact_tasks entry
        // whose contact_frame contains that word; throws if there is none
        YAML::Node contactConfig(const std::string &side) const;

        std::string yaml_config_path_;
        std::string urdf_path_;
        YAML::Node yaml_config_;

        // foot frames (contact_frame of the contact_tasks entries), used by the contact and swing tasks
        std::string left_foot_frame_;
        std::string right_foot_frame_;

        tsid::robots::RobotWrapper &robot_wrapper_; // robot model, owned by the caller
        // TSID problem (tasks, contacts, QP data) on robot_wrapper_, built in the initializer list
        tsid::InverseDynamicsFormulationAccForce tsid_;

        // task settings (setup*Task)
        TaskSetting com_setting_;
        TaskSetting base_orientation_setting_;
        TaskSetting posture_setting_;
        TaskSetting actuation_bounds_setting_;
        TaskSetting swing_setting_;
        double contact_f_max_[2] = {1000.0, 1000.0};  // max normal force [left, right] (f_max of the yaml):
                                                       // restored at landing, TSID ramps it to ~0 at liftoff
        double left_contact_force_reg_weight_ = 1e-3;  // w_force_reg of addRigidContact
        double right_contact_force_reg_weight_ = 1e-3;
        double contact_transition_time_ = 0.15;        // [s] force ramp before a contact is removed

        // tsid tasks
        std::shared_ptr<tsid::tasks::TaskComEquality> com_task_; // track CoM position
        std::shared_ptr<tsid::tasks::TaskActuationBounds> actuation_bounds_task_; //
        std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_; // actuated joint tracking task
        std::shared_ptr<tsid::tasks::TaskSE3Equality> base_orientation_task_; // track base euler (orientation)
        
        // contact taskes are added to tsid based on the motion mode (single/double support)
        std::shared_ptr<tsid::contacts::Contact6d> left_foot_contact_task;
        std::shared_ptr<tsid::contacts::Contact6d> right_foot_contact_task;        
        
        // Swing foot tasks are added to tsid also based on the planned motion
        std::shared_ptr<tsid::tasks::TaskSE3Equality> left_foot_swing_task ; 
        std::shared_ptr<tsid::tasks::TaskSE3Equality> right_foot_swing_task ;

        // Task Reference Trajectory objects
        // references, preallocated (setReference copies from them, no allocation per cycle)
        tsid::trajectories::TrajectorySample com_ref_;          // (3)
        tsid::trajectories::TrajectorySample posture_ref_;      // (na)
        tsid::trajectories::TrajectorySample base_ref_;         // (12, 6): SE3 as a 12-vector
        tsid::trajectories::TrajectorySample swing_ref_[2];     // (12, 6)

        // QP solver
        // solver (the factory returns a raw pointer: owned here)
        std::unique_ptr<tsid::solvers::SolverHQPBase> solver_;

        // tsid (QP) solution, preallocated
        Eigen::VectorXd dv_sol_, tau_sol_; // acceleration + joint torques
        Eigen::VectorXd f_contact_;   // 12: 4 corners x 3 (one contact)
        Eigen::Matrix<double, 6, 1> wrench_sol_[2]; // contact wrench 

        bool contact_active_[2] = {true, true};
        pinocchio::FrameIndex foot_id_[2];
        pinocchio::FrameIndex base_id_;
    };
} // namespace biped_controllers
