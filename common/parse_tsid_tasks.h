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
        double sole_z = -0.040;
        std::shared_ptr<tsid::contacts::Contact6d> contact = nullptr;
    };

    // Constructors
    tsidTaskParser(const std::string &yaml_config_path, const std::string &urdf_path);
    tsidTaskParser(const std::string &yaml_config_path, std::shared_ptr<tsid::robots::RobotWrapper> robot);

    // Build and register all tasks into the TSID formulation
    void setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid);
    void setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);

    // Initialize or update contact references using current robot kinematics from tsid.data()
    void updateContactReferences(const pinocchio::Data &data);

    // Getters for contact information and geometry
    const std::vector<ContactInfo>& getContactInfos() const { return m_contact_infos; }
    std::shared_ptr<tsid::contacts::Contact6d> getContact(const std::string &frame_name) const;
    double getSoleZ() const;
    double getSoleZ(const std::string &frame_name) const;

    // Helper: compute average position of contact frames in world coordinates
    Eigen::Vector3d computeSupportCenter(const pinocchio::Data &data, const tsid::robots::RobotWrapper &robot) const;

    // Getters for task access and runtime trajectory reference updates
    std::shared_ptr<tsid::tasks::TaskComEquality> getComTask() const { return m_com_task; }
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

    void loadYaml();
    void initRobotWrapper();
    void parseContactMetadata(const tsid::robots::RobotWrapper &robot);
    void setupContacts(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupComTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupActuationBoundsTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
    void setupPostureTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot);
};
