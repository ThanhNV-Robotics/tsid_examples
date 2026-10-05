#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <mujoco/mujoco.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "data_type.h"
using namespace std;

namespace utils {

struct IkRes {
  int status;
  int itr;
  Eigen::VectorXd err;
  Eigen::VectorXd jointPosRes;
};

vector<string> getJointNames(const mjModel *m);

void loadMjXml(mjModel *&m, mjData *&d, const string xml_path);
void printMjModelInfo(mjModel *m);
void setRobotInitConfiguration(mjModel *&m, mjData *&d, Eigen::VectorXd q0);

// Damped least-squares IK placing both ankle-pitch frames at the desired poses
// (relative to the base). Accepts fixed-base or free-flyer models; with a
// free-flyer the base is held at the origin. jointPosRes holds only the
// actuated joint angles.
IkRes computeIK_Leg(const pinocchio::Model &model, pinocchio::Data &data,
                    const Eigen::Matrix3d &Rdes_L,
                    const Eigen::Vector3d &Pdes_L,
                    const Eigen::Matrix3d &Rdes_R,
                    const Eigen::Vector3d &Pdes_R,
                    pinocchio::JointIndex left_foot_id = 0,
                    pinocchio::JointIndex right_foot_id = 0);

Eigen::VectorXd computeIntialStandConfig(const pinocchio::Model &model,
                               pinocchio::Data &data, double hip_width,
                               double base_height);
Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw);
void load_postureTask_gain (const string cd_yaml_path, Eigen::VectorXd &Kp, Eigen::VectorXd &Kd);

// Convert a RobotState (world-frame base twist) into Pinocchio's free-flyer
// convention: q = [pos_W, quat (x,y,z,w), qj], v = [v_B, w_B, dqj] with the
// base twist expressed in the base frame. q and v must be pre-sized (nq, nv).
void robotStateToPinocchio(const RobotState &state, Eigen::VectorXd &q,
                           Eigen::VectorXd &v);

// Quintic (5th-order, minimum-jerk) interpolation from x_start to x_end over
// duration T, with zero velocity and acceleration at both ends. For t >= T the
// output holds x_end with zero velocity and acceleration.
void quinticTrajectory(const Eigen::VectorXd &x_start,
                       const Eigen::VectorXd &x_end, double t, double T,
                       Eigen::VectorXd &x, Eigen::VectorXd &dx,
                       Eigen::VectorXd &ddx);

} // namespace utils
