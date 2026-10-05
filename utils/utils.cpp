#include "utils.h"
#include <cstdio>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace utils {

void loadMjXml(mjModel *&m, mjData *&d, const string xml_path) {
  char loadError[1024] = "";
  m = mj_loadXML(xml_path.c_str(), nullptr, loadError, sizeof(loadError));

  if (!m) {
    std::fprintf(stderr, "Failed to load MuJoCo scene '%s': %s\n",
                 xml_path.c_str(), loadError);
    d = nullptr;
    return;
  }
  d = mj_makeData(m);
}

vector<string> getJointNames(const mjModel *m) {
  vector<string> joint_names;
  if (!m)
    return joint_names;

  joint_names.reserve(m->njnt);
  for (int i = 0; i < m->njnt; ++i) {
    const char *name = mj_id2name(m, mjOBJ_JOINT, i);
    joint_names.emplace_back(name ? name
                                  : "unnamed_joint_" + std::to_string(i));
  }
  return joint_names;
}

void printMjModelInfo(mjModel *m) {
  vector<string> joint_names = getJointNames(m);
  // print joint names
  for (int i = 0; i < joint_names.size(); i++) {
    std::cout << "Joint " << i << " :" << joint_names[i] << endl;
  }
  std::cout << "Model nv: " << m->nv << endl;
  std::cout << "Model nq: " << m->nq << endl;
}

void setRobotInitConfiguration(mjModel *&m, mjData *&d, Eigen::VectorXd q0) {
  if (!m || !d)
    return;
  if (q0.size() == m->nq) {
    for (int i = 0; i < m->nq; i++) {
      d->qpos[i] = q0(i);
    }
  } else if (q0.size() < m->nq) {
    // Offset for actuated joints (e.g. 19 - 12 = 7 for floating-base biped)
    const int offset = m->nq - q0.size();
    for (int i = 0; i < q0.size(); i++) {
      d->qpos[offset + i] = q0(i);
    }
  } else {
    std::cerr << "Error: Dimension mismatch! q_0 size is " << q0.size()
              << ", but model m->nq is " << m->nq << std::endl;
    return;
  }
  for (int i = 0; i < m->nv; i++) {
    d->qvel[i] = 0.0;
  }
  mj_forward(m, d);
}

IkRes computeIK_Leg(const pinocchio::Model &model, pinocchio::Data &data,
                    const Eigen::Matrix3d &Rdes_L,
                    const Eigen::Vector3d &Pdes_L,
                    const Eigen::Matrix3d &Rdes_R,
                    const Eigen::Vector3d &Pdes_R,
                    pinocchio::JointIndex left_foot_id,
                    pinocchio::JointIndex right_foot_id) {
  const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
  const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);

  // Initial guess
  Eigen::VectorXd qIk = pinocchio::neutral(model);
  qIk[3] = 0.1;    // left knee
  qIk[9] = 0.1;    // right knee
  qIk[0] = -0.08;  // left hip pitch
  qIk[6] = -0.08;  // right hip pitch
  qIk[5] = -0.05;  // left hip roll
  qIk[11] = -0.08; // right hip roll

  // Resolve foot joint indices if not provided
  pinocchio::JointIndex J_Idx_l = model.getJointId("left_ankle_pitch_joint");
  pinocchio::JointIndex J_Idx_r = model.getJointId("right_ankle_pitch_joint");

  const double eps = 1e-4;
  const int IT_MAX = 100;
  const double DT = 7e-1;
  const double damp = 5e-3;
  Eigen::MatrixXd JL(6, model.nv);
  Eigen::MatrixXd JR(6, model.nv);
  Eigen::MatrixXd JCompact(12, model.nv);
  JL.setZero();
  JR.setZero();
  JCompact.setZero();

  bool success = false;
  Eigen::Matrix<double, 6, 1> errL, errR;
  Eigen::Matrix<double, 12, 1> errCompact;
  Eigen::VectorXd v(model.nv);

  int itr_count = 0;
  for (itr_count = 0; itr_count < IT_MAX; itr_count++) {
    pinocchio::forwardKinematics(model, data, qIk);
    const pinocchio::SE3 iMdL = data.oMi[J_Idx_l].actInv(oMdesL);
    const pinocchio::SE3 iMdR = data.oMi[J_Idx_r].actInv(oMdesR);
    errL = pinocchio::log6(iMdL).toVector(); // in joint frame
    errR = pinocchio::log6(iMdR).toVector(); // in joint frame
    errCompact.block<6, 1>(0, 0) = errL;
    errCompact.block<6, 1>(6, 0) = errR;
    if (errCompact.norm() < eps) {
      success = true;
      break;
    }

    pinocchio::computeJointJacobian(model, data, qIk, J_Idx_l,
                                    JL); // JL in joint frame
    pinocchio::computeJointJacobian(model, data, qIk, J_Idx_r,
                                    JR); // JR in joint frame

    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(model.nv, model.nv);

    pinocchio::Data::Matrix6 JlogL;
    pinocchio::Data::Matrix6 JlogR;
    pinocchio::Jlog6(iMdL.inverse(), JlogL);
    pinocchio::Jlog6(iMdR.inverse(), JlogR);
    JL = -JlogL * JL;
    JR = -JlogR * JR;
    JCompact.block(0, 0, 6, model.nv) = JL;
    JCompact.block(6, 0, 6, model.nv) = JR;

    Eigen::Matrix<double, 12, 12> JJt;
    JJt.noalias() = JCompact * W * JCompact.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
    qIk = pinocchio::integrate(model, qIk, v * DT);
  }

  IkRes res;
  res.err = errCompact;
  res.itr = itr_count;
  res.status = success ? 0 : -1;
  res.jointPosRes = qIk;
  return res;
}

Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw) {
  Eigen::Matrix<double, 3, 3> Rx, Ry, Rz;
  Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Ry << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
  Rx << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
  return Rz * Ry * Rx;
}

Eigen::VectorXd computeIntialStandConfig(const pinocchio::Model &model,
                               pinocchio::Data &data, double hip_width,
                               double base_height) {

  Eigen::Vector3d fe_l_pos_L_des = {0.0, hip_width / 2,
                                    -base_height}; // desired left feet pos
  Eigen::Vector3d fe_r_pos_L_des = {0.0, -hip_width / 2,
                                    -base_height}; // desired right feet pos

  Eigen::Vector3d fe_l_eul_L_des = {0.0, 0.0, 0.0};
  Eigen::Vector3d fe_r_eul_L_des = {0.0, 0.0, 0.0};
  Eigen::Matrix3d fe_l_rot_des =
      eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
  Eigen::Matrix3d fe_r_rot_des =
      eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

  auto resLeg = computeIK_Leg(model, data, fe_l_rot_des, fe_l_pos_L_des,
                               fe_r_rot_des, fe_r_pos_L_des);

  return resLeg.jointPosRes;
}

void load_postureTask_gain(const string cd_yaml_path, Eigen::VectorXd &Kp,
                           Eigen::VectorXd &Kd) {
  try {
    YAML::Node root = YAML::LoadFile(cd_yaml_path);
    if (!root.IsDefined() || !root.IsMap()) {
      std::cerr << "[utils::load_postureTask_gain] Error: Failed to load YAML "
                   "or root is not a map: "
                << cd_yaml_path << std::endl;
      return;
    }

    std::vector<double> kp_list;
    std::vector<double> kd_list;

    YAML::Node jg = root;
    if (root["posture_task"] && root["posture_task"]["joint_gain"]) {
      jg = root["posture_task"]["joint_gain"];
    } else if (root["posture_task"] && root["posture_task"]["joint_gains"]) {
      jg = root["posture_task"]["joint_gains"];
    }

    for (const auto &kv : jg) {
      const YAML::Node &node = kv.second;
      if (node.IsMap() && node["kp"] && node["kd"]) {
        kp_list.push_back(node["kp"].as<double>());
        kd_list.push_back(node["kd"].as<double>());
      }
    }

    const int n = static_cast<int>(kp_list.size());
    if (n == 0) {
      std::cerr << "[utils::load_postureTask_gain] Warning: No joint kp/kd "
                   "gains found in: "
                << cd_yaml_path << std::endl;
      return;
    }

    Kp = Eigen::VectorXd::Zero(n);
    Kd = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      Kp(i) = kp_list[i];
      Kd(i) = kd_list[i];
    }
  } catch (const std::exception &e) {
    std::cerr << "[utils::load_postureTask_gain] Exception while loading YAML: "
              << e.what() << std::endl;
  }
}

void robotStateToPinocchio(const RobotState &state, Eigen::VectorXd &q,
                           Eigen::VectorXd &v) {
  const Eigen::Matrix3d R = state.quat_b_W.toRotationMatrix();
  q << state.pos_b_W, state.quat_b_W.coeffs(), state.qj;
  v << R.transpose() * state.vb_W, R.transpose() * state.wb_W, state.dq_j;
}

void quinticTrajectory(const Eigen::VectorXd &x_start,
                       const Eigen::VectorXd &x_end, double t, double T,
                       Eigen::VectorXd &x, Eigen::VectorXd &dx,
                       Eigen::VectorXd &ddx) {
  const Eigen::VectorXd delta = x_end - x_start;
  if (T <= 0.0 || t >= T) {
    x = x_end;
    dx = Eigen::VectorXd::Zero(x_end.size());
    ddx = Eigen::VectorXd::Zero(x_end.size());
    return;
  }

  const double tau = std::max(t, 0.0) / T;
  const double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
  const double ds = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / T;
  const double dds = (60.0 * tau - 180.0 * std::pow(tau, 2) + 120.0 * std::pow(tau, 3)) / (T * T);

  x = x_start + s * delta;
  dx = ds * delta;
  ddx = dds * delta;
}

} // namespace utils
