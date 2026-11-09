#pragma once
#include "Eigen/Dense"

#include "common/data_type.h"

#include <cmath>
#include <string>

namespace biped_controllers
{
    // This class wraps the Lineear Inverted Pendumlumn - based planner.
    // It plans the CoM motion of the walking locomotion task; the foot motion (footholds, swing
    // trajectory) is planned by FootPlanner, which gives this planner the CP target of each phase.
    //
    // C++ port of the CoM part of tsid_examples/python/WalkPlanner.py (+ LIP.py). Call
    // planWalking() once per control cycle (period dt from the yaml). Phases of Tswing each:
    //   1. init phase (both feet on the ground): shift the capture point (CP) over the first stance
    //      foot (cp_target = that foot)
    //   2. steps: the ZMP is placed so that the CP reaches the swing foot's foothold at the end of
    //      the step (cp_target = that foothold)
    // At the end of a phase the ZMP moves onto cp_target (the foot there becomes the stance foot).
    // No double support phase between the steps (as in the Python version).
    //
    // World frame: x forward, z up. Outputs: CoM position / velocity / acceleration (TSID CoM
    // task), ZMP, CP and phase.
    class LIPTaskPlanner
    {
        public:
            // constructor: pass an .yaml file which contains the setting parameters
            // (section "LIPPlanner"); throws std::runtime_error if the file is missing or invalid
            LIPTaskPlanner(const std::string &CONFIG_YAML_PATH);

            // LIP / capture point (CP) model in the horizontal plane (x, y), w = sqrt(g / zc):
            //   CoM:  d(com)/dt = w (cp - com)      the CoM follows the CP
            //   CP:   d(cp)/dt  = w (cp - zmp)      the CP diverges away from the ZMP

            // CoM velocity for the given CP and CoM
            Vector2d CoM_dynamics (const Vector2d &cp, const Vector2d &com) const;
            // CP velocity for the given ZMP and CP
            Vector2d CP_dynamics (const Vector2d &zmp, const Vector2d &cp) const;
            // one exact step (dt) of the internal CoM, driven by the CP; returns the new CoM
            Vector2d ComputeCoM (const Vector2d &cp);
            // one exact step (dt) of the internal CP, driven by the ZMP; returns the new CP
            Vector2d ComputeCP (const Vector2d &zmp);

            // ---- walking planner ----

            // restart the plan from the robot's current state (e.g. in WBCController::on_activate):
            // CoM (x, y) at rest (CP = CoM), init phase next
            void reset(const Vector2d &com);

            // one planning cycle: ZMP for the CP target of this phase (x, y, planning frame, from
            // WalkPlanner), constant over the phase and kept in [zmp_min, zmp_max] (default: no
            // limit), then integrate the CP and the CoM
            void planWalking(const Vector2d &cp_target,
                             const Vector2d &zmp_min = Vector2d::Constant(-1e9),
                             const Vector2d &zmp_max = Vector2d::Constant(1e9));

            // one cycle with the caller's timing (WalkPlanner: steps of variable length, double
            // support): the ZMP that brings the CP (cp_fb) to cp_target in the remaining time t_rem,
            // in [zmp_min, zmp_max]; held (still kept in the limits) once t_rem <= zmp_hold_time,
            // e.g. a late step (t_rem 0). Then the CP and the CoM are integrated. The own phase of
            // planWalking() is not used
            void planCycle(const Vector2d &cp_target, double t_rem, const Vector2d &zmp_min, const Vector2d &zmp_max);

            // ---- CP feedback (tsid_examples doc/event_based_control.md, Stage 1 + 2) ----
            // measured CoM and CP (c + dc / w) of the robot in the planning frame, for the next
            // planWalking(): its ZMP drives cp_fb = cp + cp_feedback_gain (cp_meas - cp) (the planned
            // CP blended toward the measured one) to the target. Without a measurement: open loop
            void setMeasuredState(const Vector2d &com, const Vector2d &cp);
            // pull the planned CoM and CP toward the measured ones by cp_feedback_gain (touchdown:
            // the plan cannot drift away from the robot over the steps)
            void blendTowardMeasured();
            // CP at the end of the phase if the ZMP of this cycle is kept over the remaining time,
            // from cp_fb: p + e^{w t_rem} (cp_fb - p) (WalkPlanner: foothold adaptation)
            Vector2d getPredictedEndCP() const;
            // remaining time of the phase at the start of this cycle [s] (0 while held)
            double getRemainingTime() const { return this->t_rem_; }

            // LIP height [m] (default: com_height of the yaml), e.g. the CoM height of the pose the
            // robot walks in; w = sqrt(g / zc) follows. zc <= 0 is ignored
            void setComHeight(double zc);

            // re-express the LIP state (CoM, CP, ZMP, CP target) in another frame: x -> R x + t
            // (WalkPlanner: new support frame at a switch; the motion itself does not change)
            void transformState(const Eigen::Matrix2d &R, const Vector2d &t);

            // ---- reference outputs (valid after planWalking()) ----
            Vector3d getRefCoMPosition() const;     // (x, y, zc)
            Vector3d getRefCoMVelocity() const;     // (dx, dy, 0)
            Vector3d getRefCoMAcceleration() const; // (ddx, ddy, 0)

            const Vector2d &getRefZMP() const { return this->zmp_d_; }
            const Vector2d &getRefCP() const { return this->cp_; }
            // phase of the current step / init phase in [0, 1) (ContactFusion: phi); 1 while held
            double getPhase() const { return this->phi_com_; }       // planWalking() only
            bool isInitWalkingDone() const { return this->done_init_walking_; }
            // planner period [s] (1 / sampling_frequency of the yaml): call planWalking() at this rate
            double getDt() const { return this->dt_; }
            // phase (step) duration [s] (Tswing of the yaml)
            double getTswing() const { return this->T_; }
            // LIP natural frequency w = sqrt(g / zc) [1/s]
            double getOmega() const { return this->w_; }

        private:
            // constant ZMP that brings the CP from cp to cp_d_ in the time t
            Vector2d zmpForTargetCP(const Vector2d &cp, double t) const;

            double T_{1.0}; // swing time
            double dt_{0.002}; // default frequency is 500Hz
            double zc_{0.45};  // com height
            double cp_feedback_gain_{0.3}; // measured-CP blend (0 = open loop, 1 = measured CP only)
            double zmp_hold_time_{0.03};   // [s] below this remaining time the ZMP is held (ill-conditioned)

            // LIP attributes
            const double g_ = 9.81;
            double w_ = std::sqrt(g_/zc_); // natural frequency [1/s]; recomputed in the constructor

            // LIP state (horizontal, world frame), integrated by ComputeCoM() / ComputeCP()
            Vector2d com_ = Vector2d::Zero(); // planned(ref) CoM position
            Vector2d cp_ = Vector2d::Zero();  // planned(ref) capture point (CP)

            double phi_com_ = 0.0;            // phase of the current step / init phase

            Vector2d zmp_d_ = Vector2d::Zero(); // desired ZMP
            Vector2d cp_d_ = Vector2d::Zero();  // desired CP at the end of the step

            // CP feedback
            bool has_meas_ = false;              // setMeasuredState() called since reset()
            Vector2d com_meas_ = Vector2d::Zero(); // measured CoM, planning frame
            Vector2d cp_meas_ = Vector2d::Zero();  // measured CP, planning frame
            Vector2d cp_fb_ = Vector2d::Zero();    // CP the ZMP of this cycle starts from (blend)
            double t_rem_ = 0.0;                   // remaining phase time at the start of this cycle [s]

            // walking state
            bool done_init_walking_ = false;
    };

} //namespace biped_controllers
