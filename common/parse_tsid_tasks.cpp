#include "parse_tsid_tasks.h"
#include <iostream>
#include <cmath>

tsidTaskParser::tsidTaskParser(const std::string &yaml_config_path, const std::string &urdf_path)
    : m_yaml_path(yaml_config_path), m_urdf_path(urdf_path)
{
    loadYaml();
    initRobotWrapper();
    if (m_robot) {
        parseContactMetadata(*m_robot);
    }
}

tsidTaskParser::tsidTaskParser(const std::string &yaml_config_path, std::shared_ptr<tsid::robots::RobotWrapper> robot)
    : m_yaml_path(yaml_config_path), m_robot(robot)
{
    loadYaml();
    if (m_robot) {
        parseContactMetadata(*m_robot);
    }
}

void tsidTaskParser::loadYaml()
{
    try {
        m_config = YAML::LoadFile(m_yaml_path);
        std::cout << "[tsidTaskParser] Successfully loaded YAML config: " << m_yaml_path << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[tsidTaskParser] Error: Failed to parse YAML file '" << m_yaml_path 
                  << "': " << e.what() << std::endl;
    }
}

void tsidTaskParser::initRobotWrapper()
{
    if (!m_robot) {
        if (m_urdf_path.empty()) {
            std::cerr << "[tsidTaskParser] Error: URDF path is empty, cannot initialize RobotWrapper!" << std::endl;
            return;
        }
        std::vector<std::string> package_dirs;
        m_robot = std::make_shared<tsid::robots::RobotWrapper>(
            m_urdf_path, package_dirs, pinocchio::JointModelFreeFlyer(), false);
        std::cout << "[tsidTaskParser] Initialized RobotWrapper from '" << m_urdf_path 
                  << "' (nq=" << m_robot->nq() << ", nv=" << m_robot->nv() 
                  << ", na=" << m_robot->na() << ")" << std::endl;
    }
}

void tsidTaskParser::parseContactMetadata(const tsid::robots::RobotWrapper &robot)
{
    m_contact_infos.clear();
    std::vector<YAML::Node> contact_nodes;

    if (m_config["contact_tasks"]) {
        const auto &node = m_config["contact_tasks"];
        if (node.IsSequence()) {
            for (const auto &c : node) contact_nodes.push_back(c);
        } else if (node.IsMap()) {
            for (const auto &kv : node) contact_nodes.push_back(kv.second);
        }
    } else if (m_config["contacts"] && m_config["contacts"].IsMap()) {
        for (const auto &kv : m_config["contacts"]) contact_nodes.push_back(kv.second);
    } else if (m_config["contact_task"] && m_config["contact_task"].IsMap()) {
        contact_nodes.push_back(m_config["contact_task"]);
    }

    for (const auto &c_node : contact_nodes) {
        if (!c_node["contact_frame"]) continue;
        std::string frame_name = c_node["contact_frame"].as<std::string>();
        if (!robot.model().existFrame(frame_name)) {
            std::cerr << "[tsidTaskParser] Warning: Frame '" << frame_name 
                      << "' not found in robot model during metadata parsing." << std::endl;
            continue;
        }
        pinocchio::FrameIndex frame_id = robot.model().getFrameId(frame_name);
        double sole_z = c_node["sole_z"] ? c_node["sole_z"].as<double>() : -0.040;
        m_contact_infos.push_back({frame_name, frame_id, sole_z, nullptr});
    }
}

double tsidTaskParser::getSoleZ() const
{
    if (!m_contact_infos.empty()) {
        return m_contact_infos.front().sole_z;
    }
    return -0.040;
}

double tsidTaskParser::getSoleZ(const std::string &frame_name) const
{
    for (const auto &info : m_contact_infos) {
        if (info.frame_name == frame_name) {
            return info.sole_z;
        }
    }
    return getSoleZ();
}

Eigen::Vector3d tsidTaskParser::computeSupportCenter(const pinocchio::Data &data, const tsid::robots::RobotWrapper &robot) const
{
    if (m_contact_infos.empty()) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto &info : m_contact_infos) {
        center += robot.framePosition(data, info.frame_id).translation();
    }
    return center / static_cast<double>(m_contact_infos.size());
}

void tsidTaskParser::setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid)
{
    if (!m_robot) {
        std::cerr << "[tsidTaskParser] Error: RobotWrapper is null in setTaskconfig!" << std::endl;
        return;
    }
    setTaskconfig(tsid, *m_robot);
}

void tsidTaskParser::setTaskconfig(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    parseContactMetadata(robot);
    m_com_task.reset();
    m_actuation_bounds_task.reset();
    m_posture_task.reset();

    setupContacts(tsid, robot);
    setupComTask(tsid, robot);
    setupActuationBoundsTask(tsid, robot);
    setupPostureTask(tsid, robot);

    std::cout << "[tsidTaskParser] Finished configuring all tasks into TSID." << std::endl;
}

void tsidTaskParser::setupContacts(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    std::vector<YAML::Node> contact_nodes;

    if (m_config["contact_tasks"]) {
        const auto &node = m_config["contact_tasks"];
        if (node.IsSequence()) {
            for (const auto &c : node) contact_nodes.push_back(c);
        } else if (node.IsMap()) {
            for (const auto &kv : node) contact_nodes.push_back(kv.second);
        }
    } else if (m_config["contacts"] && m_config["contacts"].IsMap()) {
        for (const auto &kv : m_config["contacts"]) contact_nodes.push_back(kv.second);
    } else if (m_config["contact_task"] && m_config["contact_task"].IsMap()) {
        contact_nodes.push_back(m_config["contact_task"]);
    }

    for (size_t idx = 0; idx < contact_nodes.size() && idx < m_contact_infos.size(); ++idx) {
        const auto &c_node = contact_nodes[idx];
        auto &info = m_contact_infos[idx];

        double mu = c_node["mu"] ? c_node["mu"].as<double>() : 0.8;
        double f_min = c_node["f_min"] ? c_node["f_min"].as<double>() : 0.0;
        double f_max = c_node["f_max"] ? c_node["f_max"].as<double>() : 1000.0;
        double kp = c_node["kp"] ? c_node["kp"].as<double>() : 10.0;
        double kd = c_node["kd"] ? c_node["kd"].as<double>() : (2.0 * std::sqrt(kp));
        
        double weight = 1e-5;
        if (c_node["weight"]) {
            weight = c_node["weight"].as<double>();
        } else if (c_node["w_force_reg"]) {
            weight = c_node["w_force_reg"].as<double>();
        }

        // Contact normal (default: [0, 0, 1])
        Eigen::Vector3d contact_normal(0.0, 0.0, 1.0);
        if (c_node["contact_normal"] && c_node["contact_normal"].IsSequence()) {
            std::vector<double> n = c_node["contact_normal"].as<std::vector<double>>();
            if (n.size() == 3) {
                contact_normal = Eigen::Vector3d(n[0], n[1], n[2]).normalized();
            }
        }

        // Contact points
        Eigen::Matrix3Xd contact_points;
        if (c_node["contact_points"] && c_node["contact_points"].IsSequence()) {
            const auto &pts = c_node["contact_points"];
            const int num_pts = static_cast<int>(pts.size());
            contact_points.resize(3, num_pts);
            for (int i = 0; i < num_pts; ++i) {
                std::vector<double> pt = pts[i].as<std::vector<double>>();
                contact_points(0, i) = pt.size() > 0 ? pt[0] : 0.0;
                contact_points(1, i) = pt.size() > 1 ? pt[1] : 0.0;
                contact_points(2, i) = pt.size() > 2 ? pt[2] : 0.0;
            }
        } else {
            // Default 4 rectangular sole corners:
            double sole_z = info.sole_z;
            double x_back = c_node["sole_x_min"] ? c_node["sole_x_min"].as<double>() : -0.05;
            double x_front = c_node["sole_x_max"] ? c_node["sole_x_max"].as<double>() : 0.13;
            double y_right = c_node["sole_y_min"] ? c_node["sole_y_min"].as<double>() : -0.03;
            double y_left = c_node["sole_y_max"] ? c_node["sole_y_max"].as<double>() : 0.03;

            contact_points.resize(3, 4);
            contact_points << x_back,  x_back,  x_front, x_front,
                              y_right, y_left,  y_right, y_left,
                              sole_z,  sole_z,  sole_z,  sole_z;
        }

        std::string task_name = "contact_" + info.frame_name;
        info.contact = std::make_shared<tsid::contacts::Contact6d>(
            task_name, robot, info.frame_name, contact_points, contact_normal, mu, f_min, f_max);

        info.contact->Kp(kp * Eigen::VectorXd::Ones(6));
        info.contact->Kd(kd * Eigen::VectorXd::Ones(6));

        tsid.addRigidContact(*info.contact, weight);

        std::cout << "[tsidTaskParser] Added 6D contact for frame '" << info.frame_name 
                  << "' (kp=" << kp << ", kd=" << kd << ", w_force_reg=" << weight << ")" << std::endl;
    }
}

void tsidTaskParser::setupComTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    if (!m_config["com_task"]) {
        return;
    }

    const auto &node = m_config["com_task"];
    double kp = node["kp"] ? node["kp"].as<double>() : 50.0;
    double kd = node["kd"] ? node["kd"].as<double>() : (2.0 * std::sqrt(kp));
    double weight = node["weight"] ? node["weight"].as<double>() : 1.0;
    unsigned int priority = node["priority"] ? node["priority"].as<unsigned int>() : 1;

    m_com_task = std::make_shared<tsid::tasks::TaskComEquality>("task-com", robot);
    m_com_task->Kp(kp * Eigen::VectorXd::Ones(3));
    m_com_task->Kd(kd * Eigen::VectorXd::Ones(3));

    if (node["mask"] && node["mask"].IsSequence()) {
        std::vector<double> mask_vec = node["mask"].as<std::vector<double>>();
        if (mask_vec.size() == 3) {
            m_com_task->setMask(Eigen::Vector3d(mask_vec[0], mask_vec[1], mask_vec[2]));
        }
    } else {
        m_com_task->setMask(Eigen::Vector3d(1.0, 1.0, 0.0));
    }

    tsid.addMotionTask(*m_com_task, weight, priority);
    std::cout << "[tsidTaskParser] Added CoM task (kp=" << kp << ", kd=" << kd 
              << ", weight=" << weight << ", priority=" << priority << ")" << std::endl;
}

void tsidTaskParser::setupActuationBoundsTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    m_actuation_bounds_task = std::make_shared<tsid::tasks::TaskActuationBounds>("task-torque-bounds", robot);
    Eigen::VectorXd tau_max = Eigen::VectorXd::Zero(robot.na());

    bool found_limits = false;
    if (robot.model().effortLimit.size() >= robot.na()) {
        Eigen::VectorXd limits = robot.model().effortLimit.tail(robot.na());
        if ((limits.array() > 0.0).any()) {
            tau_max = limits;
            found_limits = true;
        }
    }

    double weight = 1.0;
    unsigned int priority = 0;

    if (m_config["actuation_bounds_task"]) {
        const auto &node = m_config["actuation_bounds_task"];
        if (node["weight"]) weight = node["weight"].as<double>();
        if (node["priority"]) priority = node["priority"].as<unsigned int>();

        if (node["tau_max"]) {
            const auto &t_node = node["tau_max"];
            if (t_node.IsScalar()) {
                tau_max.setConstant(t_node.as<double>());
                found_limits = true;
            } else if (t_node.IsSequence()) {
                std::vector<double> v = t_node.as<std::vector<double>>();
                for (size_t i = 0; i < v.size() && i < static_cast<size_t>(robot.na()); ++i) {
                    tau_max[i] = v[i];
                }
                found_limits = true;
            }
        }
    }

    if (!found_limits) {
        tau_max.setConstant(100.0);
    }

    m_actuation_bounds_task->setBounds(-tau_max, tau_max);
    tsid.addActuationTask(*m_actuation_bounds_task, weight, priority);
    std::cout << "[tsidTaskParser] Added ActuationBounds task with bounds [-" 
              << tau_max.transpose() << "]" << std::endl;
}

void tsidTaskParser::setupPostureTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    if (!m_config["posture_task"]) {
        return;
    }

    const auto &node = m_config["posture_task"];
    m_posture_task = std::make_shared<tsid::tasks::TaskJointPosture>("task-posture", robot);

    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(robot.na());
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(robot.na());

    bool found_joint_gains = false;

    // Check for joint_gain or joint_gains dictionary
    YAML::Node jg_node;
    if (node["joint_gain"]) jg_node = node["joint_gain"];
    else if (node["joint_gains"]) jg_node = node["joint_gains"];

    if (jg_node && jg_node.IsMap()) {
        int seq_idx = 0;
        for (const auto &kv : jg_node) {
            std::string j_name = kv.first.as<std::string>();
            const auto &j_val = kv.second;
            double kp_val = j_val["kp"] ? j_val["kp"].as<double>() : 100.0;
            double kd_val = j_val["kd"] ? j_val["kd"].as<double>() : 2.0 * std::sqrt(kp_val);

            int target_idx = -1;
            if (robot.model().existJointName(j_name)) {
                pinocchio::JointIndex jid = robot.model().getJointId(j_name);
                int act_idx = robot.model().idx_vs[jid] - 6;
                if (act_idx >= 0 && act_idx < robot.na()) {
                    target_idx = act_idx;
                }
            }
            if (target_idx < 0 && seq_idx < robot.na()) {
                target_idx = seq_idx;
            }

            if (target_idx >= 0 && target_idx < robot.na()) {
                Kp[target_idx] = kp_val;
                Kd[target_idx] = kd_val;
                found_joint_gains = true;
            }
            seq_idx++;
        }
    }

    if (!found_joint_gains) {
        if (node["kp"]) {
            if (node["kp"].IsScalar()) {
                Kp.setConstant(node["kp"].as<double>());
            } else if (node["kp"].IsSequence()) {
                std::vector<double> v = node["kp"].as<std::vector<double>>();
                for (size_t i = 0; i < v.size() && i < static_cast<size_t>(robot.na()); ++i) {
                    Kp[i] = v[i];
                }
            }
        } else {
            Kp.setConstant(100.0);
        }

        if (node["kd"]) {
            if (node["kd"].IsScalar()) {
                Kd.setConstant(node["kd"].as<double>());
            } else if (node["kd"].IsSequence()) {
                std::vector<double> v = node["kd"].as<std::vector<double>>();
                for (size_t i = 0; i < v.size() && i < static_cast<size_t>(robot.na()); ++i) {
                    Kd[i] = v[i];
                }
            }
        } else {
            Kd = 2.0 * Kp.cwiseSqrt();
        }
    }

    // Default to critical damping (Kd = 2 * sqrt(Kp)) unless use_critical_damping is explicitly false
    bool use_crit_damping = true;
    if (node["use_critical_damping"] && !node["use_critical_damping"].as<bool>()) {
        use_crit_damping = false;
    }
    if (use_crit_damping) {
        Kd = 2.0 * Kp.cwiseSqrt();
    }

    double weight = node["weight"] ? node["weight"].as<double>() : 1e-1;
    unsigned int priority = node["priority"] ? node["priority"].as<unsigned int>() : 1;

    m_posture_task->Kp(Kp);
    m_posture_task->Kd(Kd);
    tsid.addMotionTask(*m_posture_task, weight, priority, 0.0);

    std::cout << "[tsidTaskParser] Added Posture task (weight=" << weight << ", priority=" << priority << ")" << std::endl;
    std::cout << "[tsidTaskParser] Posture Kp gains:\n" << Kp.transpose() << std::endl;
    std::cout << "[tsidTaskParser] Posture Kd gains:\n" << Kd.transpose() << std::endl;
}

void tsidTaskParser::updateContactReferences(const pinocchio::Data &data)
{
    if (!m_robot) {
        std::cerr << "[tsidTaskParser] Error: RobotWrapper is null in updateContactReferences!" << std::endl;
        return;
    }
    for (auto &info : m_contact_infos) {
        if (info.contact) {
            info.contact->setReference(m_robot->framePosition(data, info.frame_id));
            std::cout << "[tsidTaskParser] Set initial reference for contact '" << info.frame_name << "'" << std::endl;
        }
    }
}

std::shared_ptr<tsid::contacts::Contact6d> tsidTaskParser::getContact(const std::string &frame_name) const
{
    for (const auto &info : m_contact_infos) {
        if (info.frame_name == frame_name || (info.contact && info.contact->name() == frame_name)) {
            return info.contact;
        }
    }
    return nullptr;
}
