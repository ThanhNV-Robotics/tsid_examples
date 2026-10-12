#include "estimation/cheat_state_estimator.h"

#include <iostream>
#include <stdexcept>

Cheat_StateEstimator::Cheat_StateEstimator(const mjModel* model, const mjData* data,
                                           const std::vector<std::string>& foot_bodies)
    : mj_model(model), mj_data(data) {
    if (!mj_model) return;
    for (const std::string& name : foot_bodies) {
        const int id = mj_name2id(mj_model, mjOBJ_BODY, name.c_str());
        if (id < 0) throw std::runtime_error("[Cheat_StateEstimator] unknown foot body " + name);
        foot_body_ids_.push_back(id);
        contact_force_W_.push_back(Vector3d::Zero());
    }
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

RobotState Cheat_StateEstimator::estimate(const RobotSensor& sensor) {
    RobotState state;
    state.qj = sensor.actuator_state.qj;
    state.dq_j = sensor.actuator_state.dqj;

    // Contact flags [left, right]: a foot is in contact when the vertical
    // ground-truth contact force on it exceeds contact_force_level
    computeContactForces();
    state.contact_flags.assign(contact_force_W_.size(), false);
    for (size_t i = 0; i < contact_force_W_.size(); ++i)
        state.contact_flags[i] = contact_force_W_[i].z() > contact_force_level;

    if (!mj_data || base_qpos_adr < 0) {
        this->robot_state_ = state;
        return state;
    }

    const mjtNum* qpos = mj_data->qpos + base_qpos_adr;
    const mjtNum* qvel = mj_data->qvel + base_qvel_adr;

    // MuJoCo free joint: qpos = [pos_W, quat (w,x,y,z)],
    // qvel = [linear velocity in world frame, angular velocity in base frame]
    state.pos_b_W = Eigen::Map<const Vector3d>(qpos);
    state.quat_b_W = Quat(qpos[3], qpos[4], qpos[5], qpos[6]).normalized();
    state.vb_W = Eigen::Map<const Vector3d>(qvel);
    state.wb_W = state.quat_b_W * Eigen::Map<const Vector3d>(qvel + 3);

    this->robot_state_ = state;
    return state;
}

void Cheat_StateEstimator::computeContactForces() {
    for (Vector3d& f : contact_force_W_) f.setZero();
    if (!mj_model || !mj_data) return;

    mjtNum f_c[6]; // contact force:torque in the contact frame
    for (int c = 0; c < mj_data->ncon; ++c) {
        const mjContact& con = mj_data->contact[c];
        const int b1 = mj_model->geom_bodyid[con.geom[0]];
        const int b2 = mj_model->geom_bodyid[con.geom[1]];
        for (size_t i = 0; i < foot_body_ids_.size(); ++i) {
            const int foot = foot_body_ids_[i];
            if (b1 != foot && b2 != foot) continue;
            // Skip self-collisions: the other body must not belong to the robot
            const int other = (b1 == foot) ? b2 : b1;
            if (mj_model->body_rootid[other] == mj_model->body_rootid[foot]) continue;

            mj_contactForce(mj_model, mj_data, c, f_c);
            // Contact frame rows: [normal, tangent1, tangent2] in world, normal
            // from geom[0] to geom[1]; f_c is the force on geom[1]
            const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> frame(con.frame);
            const Vector3d f_W = frame.transpose() * Eigen::Map<const Vector3d>(f_c);
            contact_force_W_[i] += (b2 == foot) ? f_W : Vector3d(-f_W);
        }
    }
}
