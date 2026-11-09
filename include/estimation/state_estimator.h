#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tsid/robots/robot-wrapper.hpp>

#include "common/data_type.h"
#include "estimation/kalman_filter.h"

// Noise and contact parameters of the estimator. Covariances are per step (not scaled by dt).
struct StateEstimatorParams
{
  // process noise Q
  Vector3d imu_process_noise_pos = Vector3d::Constant(2e-4); // base position
  Vector3d imu_process_noise_vel = Vector3d::Constant(2e-3); // base velocity
  double foot_process_noise_pos = 2e-4;  // foot position (stance); x100 in swing
  double accel_bias_process_noise = 1e-6; // accelerometer bias random walk

  // measurement noise R
  Vector3d foot_sensor_noise_pos = Vector3d::Constant(1e-3); // p_base - p_foot from leg FK
  Vector3d foot_sensor_noise_vel = Vector3d::Constant(1e-1); // v_base from leg FK + gyro
  double foot_height_sensor_noise = 1e-3;                     // p_foot.z = 0

  double initial_covariance_scale = 100.0; // P0 = scale * Q

  // foot frame origin -> sole along the world z axis [m] (v2 ankle pitch frame: 0.04)
  double foot_sole_offset = 0.04;
};

// Linear Kalman filter fusing IMU acceleration with leg kinematics (stance feet assumed fixed).
//
//   state       x = [p_B (3), v_B (3), p_foot_L (3), p_foot_R (3), accel_bias (3)]   world frame
//   measurement y = [p_B - p_foot (2x3), v_B (2x3), p_foot.z (2)]
//
// Orientation and angular velocity come directly from the IMU; joint states are passed through.
// The contact probability per foot comes from ContactFusion, which itself needs this estimate,
// so the loop runs with a one-tick delay (start in double support, p = [1, 1]):
//   se.update(sensor, contact_probability);                      // of the previous tick
//   se.getPinocchioState(q, v);
//   cf.computeDisturbanceTorque(q, v, sensor.actuator_state.torquej);
//   contact_probability = cf.fuseContactProbability(s_phi, phi);   // used by the next se.update()
// Foot kinematics use the TSID robot's pinocchio model (floating base) with its own pinocchio
// Data, so it never overwrites the data used by the TSID formulation.
class StateEstimator
{
public:
  static constexpr int numContact = 2;
  static constexpr int dimContact = 3 * numContact;              // 6
  static constexpr int dimState = 6 + dimContact + 3;            // 15
  static constexpr int dimObserve = 2 * dimContact + numContact; // 14

  // robot must outlive the estimator (its model is referenced, not copied)
  StateEstimator(const tsid::robots::RobotWrapper &robot, double dt,
                 const std::string &left_foot_frame = "left_ankle_pitch_link",
                 const std::string &right_foot_frame = "right_ankle_pitch_link",
                 const StateEstimatorParams &params = StateEstimatorParams(),
                 bool verbose = false);

  // Restart the filter for a robot standing on both feet (flat ground at z = 0): base above the
  // origin (x = y = 0) at the height where the soles touch the ground, feet from the leg
  // kinematics, zero velocity and bias, initial covariance. Call before the first update().
  void reset(const RobotSensor &sensor);

  // One filter step with the latest sensor data. Joint order of sensor.actuator_state must be the
  // pinocchio model's joint order (= robot.model().names[2..]).
  // contact_probability: per foot in [0, 1], [left, right] (ContactFusion, previous tick). It
  // scales Q / R continuously: 1 = foot fixed on the ground, 0 = swing (foot not trusted).
  void update(const RobotSensor &sensor, const Eigen::Ref<const Eigen::VectorXd> &contact_probability);

  // Estimated state (world-frame base twist)
  RobotState getEstimatedRobotState() const;
  // Same, in pinocchio / TSID convention: q = [p, quat (x,y,z,w), qj] (nq), v = [v_B, w_B, dqj]
  // with the base twist in the base frame (nv). Real-time: q, v sized nq / nv by the caller.
  void getPinocchioState(Eigen::VectorXd &q, Eigen::VectorXd &v) const;

  Vector3d getBasePosEst() const;
  Vector3d getBaseVelEst() const;
  Vector3d getAccelBiasEst() const;
  void setBasePosEst(const Vector3d &pos);

  Quat getImuQuaternion() const;
  const VectorXd &get_qj() const;  // measured joint position
  const VectorXd &get_dqj() const; // measured joint velocity
  // contact probability > 0.5, as used in the last update (no hysteresis: for the contact
  // decision itself use ContactFusion::getContactState)
  const std::vector<bool> &getContactFlags() const;
  LegState getContactState() const;

private:
  void getSensorMeasurement(const RobotSensor &sensor);

  // constructor helpers (kf_ needs its matrices in the initializer list)
  static Eigen::MatrixXd buildA(double dt);
  static Eigen::MatrixXd buildB(double dt);
  static Eigen::MatrixXd buildC();
  static Eigen::MatrixXd buildQ0(const StateEstimatorParams &p);
  static Eigen::MatrixXd buildR0(const StateEstimatorParams &p);

  const pinocchio::Model &model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex foot_frame_id_[numContact];
  Eigen::VectorXd q_fk_, v_fk_; // base at the origin: FK gives feet relative to the base

  const StateEstimatorParams params_;
  const double dt_;
  const int na_;

  // baseline covariances, scaled per contact every step (never compounded)
  const Eigen::MatrixXd Q0_, R0_;
  Eigen::MatrixXd Q_, R_;
  KalmanFilter kf_;

  // measurements
  VectorXd qj_mea_, dqj_mea_, tauj_mea_;
  Vector3d imu_accel_mea_ = Vector3d::Zero(); // IMU frame (= base frame)
  Vector3d imu_gyro_mea_ = Vector3d::Zero();  // IMU frame (= base frame)
  Quat imu_quat_ = Quat::Identity();          // base -> world

  // foot kinematics relative to the base, base frame: [left (3), right (3)]
  VectorXd foot_pos_B_ = VectorXd::Zero(dimContact);
  VectorXd foot_vel_B_ = VectorXd::Zero(dimContact);

  // measurement parts, world-aligned
  VectorXd ps_ = VectorXd::Zero(dimContact);   // p_B - p_foot
  VectorXd vs_ = VectorXd::Zero(dimContact);   // v_B
  VectorXd feet_heights_ = VectorXd::Zero(numContact);
  VectorXd y_ = VectorXd::Zero(dimObserve);    // measurement [ps_, vs_, feet_heights_]

  std::vector<bool> contact_flag_{false, false};
};
