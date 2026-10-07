#include "parse_tsid_tasks.h"
#include <tsid/math/utils.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <urdf_parser/urdf_parser.h>

tsidTaskParser::tsidTaskParser(const std::string &yaml_config_path, const std::string &urdf_path)
    : m_yaml_path(yaml_config_path), m_urdf_path(urdf_path)
{
    loadYaml();
    initRobotWrapper();
    if (m_robot) {
        parseContactMetadata(*m_robot);
    }
}

tsidTaskParser::tsidTaskParser(const std::string &yaml_config_path, std::shared_ptr<tsid::robots::RobotWrapper> robot,
                               const std::string &urdf_path)
    : m_yaml_path(yaml_config_path), m_urdf_path(urdf_path), m_robot(robot)
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

    // Every setting of a contact is read here, so the YAML node and its
    // ContactInfo can never get out of step (e.g. when an entry is skipped).
    for (const auto &c_node : contact_nodes) {
        if (!c_node["contact_frame"]) continue;
        ContactInfo info;
        info.frame_name = c_node["contact_frame"].as<std::string>();
        if (!robot.model().existFrame(info.frame_name)) {
            std::cerr << "[tsidTaskParser] Warning: Frame '" << info.frame_name
                      << "' not found in robot model, contact skipped." << std::endl;
            continue;
        }
        info.frame_id = robot.model().getFrameId(info.frame_name);

        if (c_node["mu"]) info.mu = c_node["mu"].as<double>();
        if (c_node["f_min"]) info.f_min = c_node["f_min"].as<double>();
        if (c_node["f_max"]) info.f_max = c_node["f_max"].as<double>();
        if (c_node["kp"]) info.kp = c_node["kp"].as<double>();
        info.kd = c_node["kd"] ? c_node["kd"].as<double>() : 2.0 * std::sqrt(info.kp);
        if (c_node["weight"]) info.w_force_reg = c_node["weight"].as<double>();
        else if (c_node["w_force_reg"]) info.w_force_reg = c_node["w_force_reg"].as<double>();

        // Contact normal in the contact frame (default: +z)
        if (c_node["contact_normal"] && c_node["contact_normal"].IsSequence()) {
            const std::vector<double> n = c_node["contact_normal"].as<std::vector<double>>();
            if (n.size() == 3 && Eigen::Vector3d(n[0], n[1], n[2]).norm() > 1e-9) {
                info.contact_normal = Eigen::Vector3d(n[0], n[1], n[2]).normalized();
            } else {
                std::cerr << "[tsidTaskParser] Warning: invalid contact_normal for '" << info.frame_name
                          << "', using [0, 0, 1]" << std::endl;
            }
        }

        // Contact points, in order of precedence:
        //   1. explicit "contact_points" list in the YAML
        //   2. bottom face of the link's URDF collision box ("contact_geometry: urdf", the default)
        //   3. rectangle from sole_x_min/max, sole_y_min/max, sole_z
        const std::string geometry = c_node["contact_geometry"] ? c_node["contact_geometry"].as<std::string>() : "urdf";
        if (c_node["contact_points"] && c_node["contact_points"].IsSequence()) {
            const auto &pts = c_node["contact_points"];
            info.contact_points.resize(3, pts.size());
            for (size_t i = 0; i < pts.size(); ++i) {
                const std::vector<double> pt = pts[i].as<std::vector<double>>();
                if (pt.size() != 3) {
                    throw std::runtime_error("[tsidTaskParser] contact_points of '" + info.frame_name +
                                             "' must be [x, y, z] triplets");
                }
                info.contact_points.col(i) << pt[0], pt[1], pt[2];
            }
            info.points_source = "yaml";
        } else if (geometry == "urdf" && soleCornersFromUrdf(info.frame_name, info.contact_normal, info.contact_points)) {
            info.points_source = "urdf";
        } else {
            const double sole_z = c_node["sole_z"] ? c_node["sole_z"].as<double>() : -0.040;
            const double x_back = c_node["sole_x_min"] ? c_node["sole_x_min"].as<double>() : -0.05;
            const double x_front = c_node["sole_x_max"] ? c_node["sole_x_max"].as<double>() : 0.13;
            const double y_right = c_node["sole_y_min"] ? c_node["sole_y_min"].as<double>() : -0.03;
            const double y_left = c_node["sole_y_max"] ? c_node["sole_y_max"].as<double>() : 0.03;
            info.contact_points.resize(3, 4);
            info.contact_points << x_back,  x_back,  x_front, x_front,
                                   y_right, y_left,  y_right, y_left,
                                   sole_z,  sole_z,  sole_z,  sole_z;
            info.points_source = "default";
        }

        // TSID's Contact6d is hard-wired to 4 points (only assert-checked)
        if (info.contact_points.cols() != 4) {
            throw std::runtime_error("[tsidTaskParser] Contact '" + info.frame_name + "' has " +
                                     std::to_string(info.contact_points.cols()) +
                                     " contact points; Contact6d requires exactly 4");
        }
        info.sole_z = info.contact_points.row(2).mean();

        std::cout << "[tsidTaskParser] Contact '" << info.frame_name << "' points (" << info.points_source
                  << "):\n" << info.contact_points << std::endl;
        m_contact_infos.push_back(info);
    }
}

bool tsidTaskParser::soleCornersFromUrdf(const std::string &link_name, const Eigen::Vector3d &normal,
                                          Eigen::Matrix3Xd &corners) const
{
    if (m_urdf_path.empty()) return false;
    const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(m_urdf_path);
    if (!urdf_model) {
        std::cerr << "[tsidTaskParser] Warning: failed to parse URDF '" << m_urdf_path << "'" << std::endl;
        return false;
    }
    const urdf::LinkConstSharedPtr link = urdf_model->getLink(link_name);
    if (!link) {
        std::cerr << "[tsidTaskParser] Warning: '" << link_name
                  << "' is not a URDF link, cannot read its collision geometry" << std::endl;
        return false;
    }

    // Among the link's collision boxes, take the face pointing most against the
    // contact normal (the sole); with several boxes, the one lowest along -normal.
    // URDF link frames coincide with Pinocchio's body frames, so the result is
    // directly in the contact frame.
    double best_height = std::numeric_limits<double>::infinity();
    for (const auto &collision : link->collision_array) {
        if (!collision || !collision->geometry || collision->geometry->type != urdf::Geometry::BOX) continue;
        const auto box = std::static_pointer_cast<const urdf::Box>(collision->geometry);
        const urdf::Pose &pose = collision->origin;
        const Eigen::Vector3d center(pose.position.x, pose.position.y, pose.position.z);
        const Eigen::Matrix3d R =
            Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z).toRotationMatrix();
        const Eigen::Vector3d half(box->dim.x / 2.0, box->dim.y / 2.0, box->dim.z / 2.0);

        int k = 0;
        for (int a = 1; a < 3; ++a) {
            if (std::abs(R.col(a).dot(normal)) > std::abs(R.col(k).dot(normal))) k = a;
        }
        const double s = (R.col(k).dot(normal) > 0.0) ? -1.0 : 1.0;
        const Eigen::Vector3d face_center = center + s * half(k) * R.col(k);
        const double height = face_center.dot(normal);
        if (height >= best_height) continue;
        best_height = height;

        // Corners ordered (-i,-j), (-i,+j), (+i,-j), (+i,+j) over the face's two in-plane axes
        const int i = (k == 0) ? 1 : 0;
        const int j = 3 - k - i;
        corners.resize(3, 4);
        int c = 0;
        for (double si : {-1.0, 1.0}) {
            for (double sj : {-1.0, 1.0}) {
                corners.col(c++) = face_center + si * half(i) * R.col(i) + sj * half(j) * R.col(j);
            }
        }
    }
    if (!std::isfinite(best_height)) {
        std::cerr << "[tsidTaskParser] Warning: link '" << link_name << "' has no collision box" << std::endl;
        return false;
    }
    return true;
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

Eigen::VectorXd tsidTaskParser::computeGroundedConfiguration(const Eigen::VectorXd &qj, double clearance) const
{
    if (!m_robot) {
        throw std::runtime_error("[tsidTaskParser] RobotWrapper is null in computeGroundedConfiguration");
    }
    const pinocchio::Model &model = m_robot->model();
    if (qj.size() != m_robot->na()) {
        throw std::runtime_error("[tsidTaskParser] computeGroundedConfiguration expects " +
                                 std::to_string(m_robot->na()) + " joint positions, got " +
                                 std::to_string(qj.size()));
    }

    Eigen::VectorXd q = pinocchio::neutral(model); // base at origin, identity orientation
    q.tail(m_robot->na()) = qj;

    pinocchio::Data data(model);
    pinocchio::framesForwardKinematics(model, data, q);

    // Lowest contact point in the world, using the actual sole corners so a
    // tilted foot is handled too
    double lowest_z = std::numeric_limits<double>::infinity();
    for (const auto &info : m_contact_infos) {
        const pinocchio::SE3 &oMf = data.oMf[info.frame_id];
        for (int i = 0; i < info.contact_points.cols(); ++i) {
            lowest_z = std::min(lowest_z, oMf.act(Eigen::Vector3d(info.contact_points.col(i))).z());
        }
    }
    if (!std::isfinite(lowest_z)) {
        std::cerr << "[tsidTaskParser] Warning: no contacts defined, base height left at 0" << std::endl;
        return q;
    }

    q[2] = -lowest_z + clearance;
    return q;
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
    m_base_orientation_task.reset();
    m_actuation_bounds_task.reset();
    m_posture_task.reset();

    setupContacts(tsid, robot);
    setupSwingTasks(robot);
    setupComTask(tsid, robot);
    setupBaseOrientationTask(tsid, robot);
    setupActuationBoundsTask(tsid, robot);
    setupPostureTask(tsid, robot);

    std::cout << "[tsidTaskParser] Finished configuring all tasks into TSID." << std::endl;
}

void tsidTaskParser::setupContacts(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    for (auto &info : m_contact_infos) {
        const std::string task_name = "contact_" + info.frame_name;
        info.contact = std::make_shared<tsid::contacts::Contact6d>(
            task_name, robot, info.frame_name, info.contact_points, info.contact_normal,
            info.mu, info.f_min, info.f_max);
        info.contact->Kp(info.kp * Eigen::VectorXd::Ones(6));
        info.contact->Kd(info.kd * Eigen::VectorXd::Ones(6));

        tsid.addRigidContact(*info.contact, info.w_force_reg);

        std::cout << "[tsidTaskParser] Added 6D contact for frame '" << info.frame_name
                  << "' (kp=" << info.kp << ", kd=" << info.kd << ", w_force_reg=" << info.w_force_reg << ")"
                  << std::endl;
    }
}

void tsidTaskParser::setupSwingTasks(tsid::robots::RobotWrapper &robot)
{
    // One swing task per contact foot, created here but only added to the QP
    // while that foot is in the air (startSwing / endSwing)
    if (!m_config["swing_foot_task"]) {
        return;
    }
    const auto &node = m_config["swing_foot_task"];
    const double kp = node["kp"] ? node["kp"].as<double>() : 300.0;
    const double kd = node["kd"] ? node["kd"].as<double>() : (2.0 * std::sqrt(kp));
    if (node["weight"]) m_swing_weight = node["weight"].as<double>();
    if (node["priority"]) m_swing_priority = node["priority"].as<unsigned int>();
    if (node["contact_transition_time"]) m_contact_transition_time = node["contact_transition_time"].as<double>();

    for (auto &info : m_contact_infos) {
        info.swing_task = std::make_shared<tsid::tasks::TaskSE3Equality>("swing_" + info.frame_name, robot,
                                                                         info.frame_name);
        info.swing_task->Kp(kp * Eigen::VectorXd::Ones(6));
        info.swing_task->Kd(kd * Eigen::VectorXd::Ones(6));
        info.swinging = false;
    }
    std::cout << "[tsidTaskParser] Created swing foot tasks (kp=" << kp << ", kd=" << kd << ", weight="
              << m_swing_weight << ", contact transition " << m_contact_transition_time << " s)" << std::endl;
}

tsidTaskParser::ContactInfo *tsidTaskParser::findContactInfo(const std::string &frame_name)
{
    for (auto &info : m_contact_infos) {
        if (info.frame_name == frame_name) return &info;
    }
    return nullptr;
}

void tsidTaskParser::setSwingFoot(tsid::InverseDynamicsFormulationAccForce &tsid, const std::string &air_frame,
                                  const pinocchio::Data &data)
{
    for (auto &info : m_contact_infos) {
        const bool should_swing = (info.frame_name == air_frame);
        if (should_swing && !info.swinging) {
            startSwing(tsid, info.frame_name);
            std::cout << "[Swing] liftoff " << info.frame_name << std::endl;
        } else if (!should_swing && info.swinging) {
            endSwing(tsid, info.frame_name, data);
            std::cout << "[Swing] touchdown " << info.frame_name << std::endl;
        }
    }
}

void tsidTaskParser::startSwing(tsid::InverseDynamicsFormulationAccForce &tsid, const std::string &frame_name)
{
    ContactInfo *info = findContactInfo(frame_name);
    if (!info || !info->contact || !info->swing_task || info->swinging) return;
    tsid.removeRigidContact(info->contact->name(), m_contact_transition_time);
    tsid.addMotionTask(*info->swing_task, m_swing_weight, m_swing_priority);
    info->swinging = true;
}

void tsidTaskParser::endSwing(tsid::InverseDynamicsFormulationAccForce &tsid, const std::string &frame_name,
                              const pinocchio::Data &data)
{
    ContactInfo *info = findContactInfo(frame_name);
    if (!info || !info->contact || !info->swing_task || !info->swinging) return;
    tsid.removeTask(info->swing_task->name());
    // TSID ramps the max normal force to ~0 while unloading a contact and never
    // restores it; without this the landed foot could carry (almost) no load
    info->contact->setMaxNormalForce(info->f_max);
    info->contact->setReference(m_robot->framePosition(data, info->frame_id));
    tsid.addRigidContact(*info->contact, info->w_force_reg);
    info->swinging = false;
}

void tsidTaskParser::setSwingReference(const std::string &frame_name, const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
                                       const Eigen::Matrix3d &R_W)
{
    ContactInfo *info = findContactInfo(frame_name);
    if (!info || !info->swing_task) return;
    // SE3 sample: 12 values (translation + rotation matrix), 6 derivatives;
    // TSID expects the reference velocity/acceleration in world-aligned axes
    tsid::trajectories::TrajectorySample sample(12, 6);
    Eigen::VectorXd pos_vec(12);
    tsid::math::SE3ToVector(pinocchio::SE3(R_W, pos), pos_vec);
    Eigen::VectorXd vel6 = Eigen::VectorXd::Zero(6), acc6 = Eigen::VectorXd::Zero(6);
    vel6.head<3>() = vel;
    acc6.head<3>() = acc;
    sample.setValue(pos_vec);
    sample.setDerivative(vel6);
    sample.setSecondDerivative(acc6);
    info->swing_task->setReference(sample);
}

bool tsidTaskParser::isSwinging(const std::string &frame_name) const
{
    for (const auto &info : m_contact_infos) {
        if (info.frame_name == frame_name) return info.swinging;
    }
    return false;
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

void tsidTaskParser::setupBaseOrientationTask(tsid::InverseDynamicsFormulationAccForce &tsid, tsid::robots::RobotWrapper &robot)
{
    if (!m_config["base_orientation_task"]) {
        return;
    }

    const auto &node = m_config["base_orientation_task"];
    const std::string frame = node["frame"] ? node["frame"].as<std::string>() : "link0_torso";
    if (!robot.model().existFrame(frame)) {
        std::cerr << "[tsidTaskParser] Warning: base frame '" << frame
                  << "' not found, base orientation task skipped" << std::endl;
        return;
    }
    m_base_frame_id = robot.model().getFrameId(frame);

    double kp = node["kp"] ? node["kp"].as<double>() : 100.0;
    double kd = node["kd"] ? node["kd"].as<double>() : (2.0 * std::sqrt(kp));
    double weight = node["weight"] ? node["weight"].as<double>() : 1.0;
    unsigned int priority = node["priority"] ? node["priority"].as<unsigned int>() : 1;

    // Mask over [x, y, z, roll, pitch, yaw] of the frame error, expressed in the
    // base frame; default keeps only roll and pitch (base upright, position and
    // heading free)
    Eigen::VectorXd mask(6);
    mask << 0, 0, 0, 1, 1, 0;
    if (node["mask"] && node["mask"].IsSequence()) {
        std::vector<double> m = node["mask"].as<std::vector<double>>();
        if (m.size() == 6) {
            mask = Eigen::Map<Eigen::VectorXd>(m.data(), 6);
        } else {
            std::cerr << "[tsidTaskParser] Warning: base_orientation_task mask needs 6 values, using default" << std::endl;
        }
    }

    m_base_orientation_task = std::make_shared<tsid::tasks::TaskSE3Equality>("task-base-orientation", robot, frame);
    m_base_orientation_task->Kp(kp * Eigen::VectorXd::Ones(6));
    m_base_orientation_task->Kd(kd * Eigen::VectorXd::Ones(6));
    m_base_orientation_task->setMask(mask);
    // Upright with zero yaw until updateBaseOrientationReference() sets the actual heading
    m_base_orientation_task->setReference(pinocchio::SE3::Identity());

    tsid.addMotionTask(*m_base_orientation_task, weight, priority);
    std::cout << "[tsidTaskParser] Added base orientation task on '" << frame << "' (kp=" << kp
              << ", kd=" << kd << ", weight=" << weight << ", priority=" << priority
              << ", mask=[" << mask.transpose() << "])" << std::endl;
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
    updateBaseOrientationReference(data);
    for (auto &info : m_contact_infos) {
        if (info.contact) {
            info.contact->setReference(m_robot->framePosition(data, info.frame_id));
            std::cout << "[tsidTaskParser] Set initial reference for contact '" << info.frame_name << "'" << std::endl;
        }
    }
}

void tsidTaskParser::updateBaseOrientationReference(const pinocchio::Data &data)
{
    if (!m_robot || !m_base_orientation_task) return;
    // Upright (zero roll and pitch) with the current yaw, so the masked yaw
    // does not leak into the roll/pitch error; translation is masked out
    const pinocchio::SE3 base = m_robot->framePosition(data, m_base_frame_id);
    const double yaw = std::atan2(base.rotation()(1, 0), base.rotation()(0, 0));
    const pinocchio::SE3 ref(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(), base.translation());
    m_base_orientation_task->setReference(ref);
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
