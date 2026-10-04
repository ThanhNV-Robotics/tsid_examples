#include "cheat_state_estimator.h"

#include <iostream>

Cheat_StateEstimator::Cheat_StateEstimator(const mjModel* model, const mjData* data)
    : mj_model(model), mj_data(data) {
    if (!mj_model) return;
    for (int j = 0; j < mj_model->njnt; ++j) {
        if (mj_model->jnt_type[j] == mjJNT_FREE) {
            base_qpos_adr = mj_model->jnt_qposadr[j];
            base_qvel_adr = mj_model->jnt_dofadr[j];
            break;
        }
    }
    if (base_qpos_adr < 0) {
        std::cerr << "[Cheat_StateEstimator] No free joint found; base state is left at identity/zero.\n";
    }
}

RobotState Cheat_StateEstimator::estimate(const RobotSensor& sensor) const {
    RobotState state;
    state.qj = sensor.actuator_state.qj;
    state.dq_j = sensor.actuator_state.dqj;

    if (!mj_data || base_qpos_adr < 0) return state;

    const mjtNum* qpos = mj_data->qpos + base_qpos_adr;
    const mjtNum* qvel = mj_data->qvel + base_qvel_adr;

    // MuJoCo free joint: qpos = [pos_W, quat (w,x,y,z)],
    // qvel = [linear velocity in world frame, angular velocity in base frame]
    state.pos_b_W = Eigen::Map<const Vector3d>(qpos);
    state.quat_b_W = Quat(qpos[3], qpos[4], qpos[5], qpos[6]).normalized();
    state.vb_W = Eigen::Map<const Vector3d>(qvel);
    state.wb_W = state.quat_b_W * Eigen::Map<const Vector3d>(qvel + 3);

    return state;
}
