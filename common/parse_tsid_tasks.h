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

class tsidTaskParser 
{
public:
    struct ContactInfo {
        std::string frame_name;
        pinocchio::FrameIndex frame_id;
        double sole_z = -0.040;                  // mean z of the contact points in the frame
        Eigen::Matrix3Xd contact_points;         // 3x4 sole corners in the contact frame
        Eigen::Vector3d contact_normal{0.0, 0.0, 1.0};
        std::string points_source;               // "yaml", "urdf" or "default"
        double mu = 0.8, f_min = 0.0, f_max = 1000.0;
        double kp = 10.0, kd = 2.0 * std::sqrt(10.0);
        double w_force_reg = 1e-5;
        std::shared_ptr<tsid::contacts::Contact6d> contact = nullptr;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> swing_task = nullptr; // tracks the foot while in the air
        bool swinging = false; // contact removed, swing task active
    };

    // Constructors
    tsidTaskParser(const std::string &yaml_config_path, const std::string &urdf_path);
    // urdf_path is optional; when given, contact sole corners can be read from the
    // URDF collision geometry (see contact_geometry in the YAML)
    tsidTaskParser(const std::string &yaml_config_path, std::shared_ptr<tsid::robots::RobotWrapper> robot,
                   const std::string &urdf_path = "");

    // Build and register all tasks into the TSID formulation
    void setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid);
    void setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);

    // Initialize or update contact references using current robot kinematics from tsid.data()
    void updateContactReferences(const pinocchio::Data &data);

    // Set the base orientation reference to upright (zero roll and pitch) with
    // the base's current yaw; called by updateContactReferences() as well
    void updateBaseOrientationReference(const pinocchio::Data &data);

    // ---- Swing foot / contact switching ("swing_foot_task:" YAML block) ----
    // Lift-off: unload the foot's contact over contact_transition_time (TSID
    // ramps its max force to zero, then removes it) and add its swing task
    void startSwing(tsid::InverseDynamicsFormulationAccForce &tsid, const std::string &frame_name);
    // Touchdown: remove the swing task and plant the contact again at the
    // foot's current pose
    void endSwing(tsid::InverseDynamicsFormulationAccForce &tsid, const std::string &frame_name,
                  const pinocchio::Data &data);
    // Swing reference: position, velocity and acceleration in world-aligned
    // axes, foot orientation R_W (angular velocity/acceleration zero)
    void setSwingReference(const std::string &frame_name, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                           const Eigen::Vector3d &acc, const Eigen::Matrix3d &R_W);
    bool isSwinging(const std::string &frame_name) const;

    // Getters for contact information and geometry
    const std::vector<ContactInfo>& getContactInfos() const { return m_contact_infos; }
    std::shared_ptr<tsid::contacts::Contact6d> getContact(const std::string &frame_name) const;
    double getSoleZ() const;
    double getSoleZ(const std::string &frame_name) const;

    // Full free-flyer configuration [base pos, quat (x,y,z,w), qj] with the base
    // upright at x = y = 0 and its height chosen so that the lowest contact
    // point of all contacts sits `clearance` above the floor (z = 0)
    Eigen::VectorXd computeGroundedConfiguration(const Eigen::VectorXd &qj, double clearance = 1e-3) const;

    // Helper: compute average position of contact frames in world coordinates
    Eigen::Vector3d computeSupportCenter(const pinocchio::Data &data, const tsid::robots::RobotWrapper &robot) const;

    // Getters for task access and runtime trajectory reference updates
    std::shared_ptr<tsid::tasks::TaskComEquality> getComTask() const { return m_com_task; }
    std::shared_ptr<tsid::tasks::TaskSE3Equality> getBaseOrientationTask() const { return m_base_orientation_task; }
    std::shared_ptr<tsid::tasks::TaskJointPosture> getPostureTask() const { return m_posture_task; }
    std::shared_ptr<tsid::tasks::TaskActuationBounds> getActuationBoundsTask() const { return m_actuation_bounds_task; }
    std::shared_ptr<tsid::robots::RobotWrapper> getRobot() const { return m_robot; }

    const YAML::Node& getConfig() const { return m_config; }

private:
    std::string m_yaml_path;
    std::string m_urdf_path;
    YAML::Node m_config;

    std::shared_ptr<tsid::robots::RobotWrapper> m_robot;
    std::vector<ContactInfo> m_contact_infos;
    std::shared_ptr<tsid::tasks::TaskComEquality> m_com_task;
    std::shared_ptr<tsid::tasks::TaskActuationBounds> m_actuation_bounds_task;
    std::shared_ptr<tsid::tasks::TaskJointPosture> m_posture_task;
    std::shared_ptr<tsid::tasks::TaskSE3Equality> m_base_orientation_task;
    pinocchio::FrameIndex m_base_frame_id{0};

    void loadYaml();
    void initRobotWrapper();
    void parseContactMetadata(const tsid::robots::RobotWrapper &robot);
    bool soleCornersFromUrdf(const std::string &link_name, const Eigen::Vector3d &normal,
                             Eigen::Matrix3Xd &corners) const;
    void setupContacts(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupComTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupSwingTasks(tsid::robots::RobotWrapper &robot);
    ContactInfo *findContactInfo(const std::string &frame_name);
    double m_swing_weight{10.0};
    unsigned int m_swing_priority{1};
    double m_contact_transition_time{0.05};
    void setupBaseOrientationTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupActuationBoundsTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupPostureTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
};
