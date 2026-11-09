#include "estimation/state_estimator.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "common/utils.h"

// constructor
StateEstimator::StateEstimator(const tsid::robots::RobotWrapper &robot, double dt,
                               const std::string &left_foot_frame,
                               const std::string &right_foot_frame,
                               const StateEstimatorParams &params, bool verbose)
: model_(robot.model())
, data_(robot.model())
, params_(params)
, dt_(dt)
, na_(robot.na())
, Q0_(buildQ0(params))
, R0_(buildR0(params))
, Q_(Q0_)
, R_(R0_)
, kf_(buildA(dt), buildB(dt), buildC(), params.initial_covariance_scale * Q0_, R0_, Q0_)
, qj_mea_(VectorXd::Zero(robot.na()))
, dqj_mea_(VectorXd::Zero(robot.na()))
, tauj_mea_(VectorXd::Zero(robot.na()))
{
  if (robot.is_fixed_base()) {
    throw std::invalid_argument("StateEstimator: the robot model needs a floating base");
  }

  const std::string frames[numContact] = {left_foot_frame, right_foot_frame};
  for (int i = 0; i < numContact; ++i) {
    if (!model_.existFrame(frames[i])) {
      throw std::invalid_argument("StateEstimator: frame '" + frames[i] + "' not in the model");
    }
    foot_frame_id_[i] = model_.getFrameId(frames[i]);
  }

  // base at the origin, identity orientation, zero twist: frame placements / velocities are then
  // relative to the base, expressed in the base frame
  q_fk_ = pinocchio::neutral(model_);
  v_fk_ = VectorXd::Zero(model_.nv);

  if (verbose) {
    // print out kalman filter paramter for checking
    std::cout << "dim_state = " << dimState << std::endl;
    std::cout << "A =\n" << buildA(dt) << std::endl;
    std::cout << "B =\n" << buildB(dt) << std::endl;
    std::cout << "C =\n" << buildC() << std::endl;
    std::cout << "Q0 =\n" << Q0_ << std::endl;
    std::cout << "P0 =\n" << kf_.get_P() << std::endl;
    std::cout << "R0 =\n" << R0_ << std::endl;
  }
}

Eigen::MatrixXd StateEstimator::buildA(double dt) {
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dimState, dimState);
  A.block<3, 3>(0, 3) = dt * I;
  // accelerometer bias couples into position/velocity the same way measured
  // accel does via B, just with a minus sign (true accel = measured - bias)
  // and a random-walk (identity) block on its own diagonal, already set
  // above. Baking this into A (rather than subtracting bias ad hoc in
  // update()) keeps the P = A*P*A^T + Q covariance propagation
  // consistent with the state propagation, so the filter actually learns
  // the bias/position/velocity correlation instead of it staying at zero.
  A.block<3, 3>(0, dimState - 3) = -0.5 * dt * dt * I; // propagate effect of bias to position
  A.block<3, 3>(3, dimState - 3) = -dt * I;            // propagate effect of bias to velocity
  return A;
}

Eigen::MatrixXd StateEstimator::buildB(double dt) {
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dimState, 3);
  B.block<3, 3>(0, 0) = 0.5 * dt * dt * I; // standard constant-accel position term; must match A's bias-position coupling above
  B.block<3, 3>(3, 0) = dt * I;
  return B;
}

Eigen::MatrixXd StateEstimator::buildC() {
  // measurement vector, stacked per contact i = 0..numContact-1:
  //   rows [0            : dimContact)            : p_base - p_foot_i = [ I_3, 0, -I_3 ]
  //   rows [dimContact   : 2*dimContact)          : v_base            = [ 0, I_3,  0   ]
  //   rows [2*dimContact : 2*dimContact+numContact) : p_foot_i.z      = [ 0, 0,  H ]
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dimObserve, dimState);
  for (int i = 0; i < numContact; ++i) {
    // position-residual rows for foot i: [ I_3, 0, -I_3 ]
    C.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
    C.block<3, 3>(3 * i, 6 + 3 * i) = -Eigen::Matrix3d::Identity();
    // velocity-residual rows for foot i: [ 0, I_3, 0 ]
    C.block<3, 3>(dimContact + 3 * i, 3) = Eigen::Matrix3d::Identity();
    // height-residual row for foot i: H = [0, 0, 1] picking foot_i_pos.z
    C(2 * dimContact + i, 6 + 3 * i + 2) = 1.0;
  }
  return C;
}

Eigen::MatrixXd StateEstimator::buildQ0(const StateEstimatorParams &p) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(dimState, dimState);
  Q.block<3, 3>(0, 0) = p.imu_process_noise_pos.asDiagonal();
  Q.block<3, 3>(3, 3) = p.imu_process_noise_vel.asDiagonal();
  for (int i = 0; i < numContact; ++i) {
    Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) *= p.foot_process_noise_pos;
  }
  // accelerometer bias drifts slowly, so its random-walk process noise
  // should be small relative to the other states
  Q.block<3, 3>(dimState - 3, dimState - 3) *= p.accel_bias_process_noise;
  return Q;
}

Eigen::MatrixXd StateEstimator::buildR0(const StateEstimatorParams &p) {
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dimObserve, dimObserve);
  for (int i = 0; i < numContact; ++i) {
    R.block<3, 3>(3 * i, 3 * i) = p.foot_sensor_noise_pos.asDiagonal();
    R.block<3, 3>(dimContact + 3 * i, dimContact + 3 * i) = p.foot_sensor_noise_vel.asDiagonal();
  }
  R.block(2 * dimContact, 2 * dimContact, numContact, numContact) *= p.foot_height_sensor_noise;
  return R;
}

void StateEstimator::reset(const RobotSensor &sensor) {
  // foot positions relative to the base (base frame) and the IMU orientation
  getSensorMeasurement(sensor);
  const Eigen::Matrix3d R_wb = imu_quat_.toRotationMatrix();

  // sole i in the world (the measurement model of update()): p_B + R_wb p_f_i - offset e_z.
  // Base above the origin, at the height where the mean sole height is 0
  Vector3d p_base = Vector3d::Zero();
  for (int i = 0; i < numContact; ++i) {
    p_base.z() -= (R_wb * foot_pos_B_.segment<3>(3 * i)).z() / numContact;
  }
  p_base.z() += params_.foot_sole_offset;

  // state x = [p_B, v_B = 0, sole left, sole right, bias = 0]; init() also resets P
  Eigen::VectorXd x = Eigen::VectorXd::Zero(dimState);
  x.segment<3>(0) = p_base;
  for (int i = 0; i < numContact; ++i) {
    x.segment<3>(6 + 3 * i) = p_base + R_wb * foot_pos_B_.segment<3>(3 * i);
    x(6 + 3 * i + 2) -= params_.foot_sole_offset;
  }
  kf_.init(x);

  // standing on both feet, baseline covariances
  Q_ = Q0_;
  R_ = R0_;
  contact_flag_.assign(numContact, true);
}

void StateEstimator::update(const RobotSensor &sensor,
                            const Eigen::Ref<const Eigen::VectorXd> &contact_probability) {

  if (contact_probability.size() != numContact) {
    throw std::invalid_argument("StateEstimator::update: contact_probability needs one value per foot");
  }

  getSensorMeasurement(sensor);

  // smooth contact confidence in [0,1] (fused probability) instead of a hard
  // flag, so Q/R scaling ramps continuously through touchdown/liftoff
  double contact_conf[numContact];
  for (int i = 0; i < numContact; ++i) {
    contact_conf[i] = std::clamp(contact_probability(i), 0.0, 1.0);
    // keep the boolean flag for getContactFlags()/external reporting
    contact_flag_[i] = contact_conf[i] > 0.5;
  }

  const Eigen::Matrix3d R_wb = imu_quat_.toRotationMatrix();

  for (int i = 0; i < numContact; ++i) {
    const int qIndex = 6 + 3 * i;               // i-th foot position in x
    const int rIndex1 = 3 * i;                  // i-th foot position residual in y
    const int rIndex2 = dimContact + 3 * i;     // i-th foot velocity residual in y
    const int rIndex3 = 2 * dimContact + i;     // i-th foot height residual in y

    const double high_suspect_number = 100;
    // linear blend between 1 (full contact confidence) and high_suspect_number
    // (no contact): a stance foot is trusted to stay put (small Q, small R),
    // a swing foot is not (large Q, large R).
    // NOTE: scale from the fixed baseline Q0_/R0_, not from Q_/R_'s own
    // current value, otherwise the scaling compounds every call and the
    // covariance diverges after a few steps of a foot staying out of contact
    const double weight = high_suspect_number + contact_conf[i] * (1.0 - high_suspect_number);

    Q_.block<3, 3>(qIndex, qIndex) = weight * Q0_.block<3, 3>(qIndex, qIndex);
    R_.block<3, 3>(rIndex1, rIndex1) = weight * R0_.block<3, 3>(rIndex1, rIndex1);
    R_.block<3, 3>(rIndex2, rIndex2) = weight * R0_.block<3, 3>(rIndex2, rIndex2);
    R_(rIndex3, rIndex3) = weight * R0_(rIndex3, rIndex3);

    const Vector3d p_f = foot_pos_B_.segment<3>(3 * i);
    const Vector3d v_f = foot_vel_B_.segment<3>(3 * i);
    ps_.segment<3>(3 * i) = -R_wb * p_f;
    ps_(3 * i + 2) += params_.foot_sole_offset; // foot state = sole point, not frame origin
    vs_.segment<3>(3 * i) = -R_wb * (v_f + imu_gyro_mea_.cross(p_f));

    // height-residual target stays 0 (sole on flat ground). Alternative: blend
    // by contact confidence between ground height and the current estimate so a
    // swinging foot is not pulled toward the ground:
    // feet_heights_[i] = (1.0 - contact_conf[i]) * kf_.get_state()(qIndex + 2);
  }
  kf_.set_Q(Q_);
  kf_.set_R(R_);

  // body acceleration in the world frame; A's bias-coupling blocks subtract the
  // current bias estimate, so no separate "accel - bias" term is needed here
  const Vector3d g(0, 0, -9.81);
  const Vector3d accel = R_wb * imu_accel_mea_ + g;

  y_ << ps_, vs_, feet_heights_;

  kf_.predict(accel);
  kf_.update(y_);
}

void StateEstimator::getSensorMeasurement(const RobotSensor &sensor) {
  qj_mea_ = sensor.actuator_state.qj;
  dqj_mea_ = sensor.actuator_state.dqj;
  tauj_mea_ = sensor.actuator_state.torquej;

  imu_accel_mea_ = sensor.imu_sensor.imu_accel_L;
  imu_gyro_mea_ = sensor.imu_sensor.imu_gyro_L;
  imu_quat_ = sensor.imu_sensor.imu_quat_.normalized();

  // foot position and velocity relative to the base, in the base frame
  q_fk_.tail(na_) = qj_mea_;
  v_fk_.tail(na_) = dqj_mea_;
  pinocchio::forwardKinematics(model_, data_, q_fk_, v_fk_);
  pinocchio::updateFramePlacements(model_, data_);
  for (int i = 0; i < numContact; ++i) {
    foot_pos_B_.segment<3>(3 * i) = data_.oMf[foot_frame_id_[i]].translation();
    foot_vel_B_.segment<3>(3 * i) =
        pinocchio::getFrameVelocity(model_, data_, foot_frame_id_[i], pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
  }
}

RobotState StateEstimator::getEstimatedRobotState() const {
  const Eigen::VectorXd x = kf_.get_state();
  RobotState state;
  state.pos_b_W = x.segment<3>(0); // KF-estimated base position (world frame)
  state.quat_b_W = imu_quat_;
  state.qj = qj_mea_; // joint position is measured directly, not filtered
  state.vb_W = x.segment<3>(3); // KF-estimated base linear velocity (world frame)
  state.wb_W = imu_quat_.toRotationMatrix() * imu_gyro_mea_; // gyro (base frame) rotated to world
  state.dq_j = dqj_mea_; // joint velocity is measured directly, not filtered
  state.contact_flags = contact_flag_;
  return state;
}

void StateEstimator::getPinocchioState(Eigen::VectorXd &q, Eigen::VectorXd &v) const {
  // written straight from the members: going through getEstimatedRobotState() (as
  // utils::robotStateToPinocchio) would copy the joint vectors into a RobotState every call
  q.resize(model_.nq); // no-op when already sized (no allocation)
  v.resize(model_.nv);
  const Eigen::VectorXd &x = kf_.get_state();
  const Eigen::Matrix3d R_wb = imu_quat_.toRotationMatrix();

  // q = [p_B (world), quat (x, y, z, w), qj]
  q.head<3>() = x.segment<3>(0);
  q.segment<4>(3) = imu_quat_.coeffs();
  q.tail(na_) = qj_mea_;

  // v = [v_B, w_B, dqj], base twist in the base frame (w_B: the gyro, rotated to the world and
  // back as in getEstimatedRobotState)
  v.head<3>().noalias() = R_wb.transpose() * x.segment<3>(3);
  v.segment<3>(3).noalias() = R_wb.transpose() * (R_wb * imu_gyro_mea_);
  v.tail(na_) = dqj_mea_;
}

Vector3d StateEstimator::getBasePosEst() const {
  return kf_.get_state().segment<3>(0);
}

Vector3d StateEstimator::getBaseVelEst() const {
  return kf_.get_state().segment<3>(3);
}

Vector3d StateEstimator::getAccelBiasEst() const {
  return kf_.get_state().segment<3>(dimState - 3);
}

void StateEstimator::setBasePosEst(const Vector3d &pos) {
  Eigen::VectorXd x = kf_.get_state();
  x.segment<3>(0) = pos;
  kf_.set_state(x);
}

Quat StateEstimator::getImuQuaternion() const {
  return imu_quat_;
}

const VectorXd &StateEstimator::get_qj() const {
  return qj_mea_;
}

const VectorXd &StateEstimator::get_dqj() const {
  return dqj_mea_;
}

const std::vector<bool> &StateEstimator::getContactFlags() const {
  return contact_flag_;
}

LegState StateEstimator::getContactState() const {
  if (contact_flag_[0] && contact_flag_[1])
    return DSt;
  else if (contact_flag_[0])
    return LSt;
  else if (contact_flag_[1])
    return RSt;
  else
    return DSt; // fallback when neither foot is in contact (e.g., flight phase)
}
