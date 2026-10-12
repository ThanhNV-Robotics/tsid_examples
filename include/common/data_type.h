// Author: Van Thanh Nguyen


#pragma once
#include <Eigen/Dense>

// alias for convinient uses
// Vector types
using VectorXd  = Eigen::VectorXd;
using Vector2d  = Eigen::Vector2d;
using Vector3d  = Eigen::Vector3d;
using Vector4d  = Eigen::Vector4d;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
// Matrix types
using MatrixXd  = Eigen::MatrixXd;
using Matrix2d  = Eigen::Matrix2d;
using Matrix3d  = Eigen::Matrix3d;
using Matrix4d  = Eigen::Matrix4d;

// Rotation / Transform
using Quat  = Eigen::Quaterniond; // q = w + xi + yj +zk;
using AngleAxis = Eigen::AngleAxisd;

// Jacobian matrix
// Pinocchio frame Jacobian: 6 rows (3 linear + 3 angular), nv cols (dynamic)
using Jacobian6 = Eigen::Matrix<double, 6, Eigen::Dynamic>; // 6 x nv
using Jacobian3 = Eigen::Matrix<double, 3, Eigen::Dynamic>; // 3 x nv, mostly used for CoM Jacobian
using JacobianX = Eigen::MatrixXd;                          // general dynamic Jacobian

struct ActuatorState
{
    int robot_na{0}; // number of actuated joint

    VectorXd qj; // joint position vector
    VectorXd dqj; // joint velocity vector
    VectorXd torquej; // joint torque 

    // constructor
    ActuatorState (int naIn = 0) : robot_na(naIn)
    {
        // init member variables
        qj = VectorXd::Zero(robot_na);
        dqj = VectorXd::Zero(robot_na);
        torquej = VectorXd::Zero(robot_na);
    }
};

struct IMUSensor
{
    Vector3d imu_accel_L = Vector3d::Zero(); // imu acceleration, in local imu frame
    Vector3d imu_gyro_L = Vector3d::Zero(); // imu angular velocity/ gyroscope in local frame
    Quat  imu_quat_ = Quat::Identity(); // imu quaternion
};

struct RobotSensor
{
    ActuatorState actuator_state;
    IMUSensor imu_sensor;

    double left_touch_sensor;
    double right_touch_sensor; 

    RobotSensor (int naIn = 0) : actuator_state(naIn)
    {
    }
};

struct RobotState
{
    // position
    Vector3d pos_b_W = Vector3d::Zero();  // base position, in world frame
    Quat quat_b_W = Quat::Identity();  // base quaternion, in world frame
    VectorXd qj ; // actuated joint position

    // velocity
    Vector3d vb_W = Vector3d::Zero(); // base linear velocity, w.r.t world frame
    Vector3d wb_W = Vector3d::Zero(); // base angular velocity, w.r.t world frame
    VectorXd dq_j ;

    // contact state
    std::vector<bool> contact_flags {false, false};
};

enum LegState
{
    LSt, // left leg is stance/support leg
    RSt, // right leg is stance/support leg
    DSt // double support no use but reserverd
};

enum MotionState
{
    STAND,
    WARM_UP,
    WALK_TO_STAND,
    WALK
};