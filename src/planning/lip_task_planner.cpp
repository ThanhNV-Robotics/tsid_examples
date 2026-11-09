#include "planning/lip_task_planner.h"
#include "common/data_type.h"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace biped_controllers

{
    // Reads the planner settings from the "LIPPlanner" section of a yaml file, e.g.
    //   LIPPlanner:
    //     Tswing: 1.0               # [s] swing (step) duration
    //     sampling_frequency: 500   # [Hz] planner rate -> dt = 1 / f
    //     com_height: 0.45          # [m] CoM height of the LIP model
    //     cp_feedback_gain: 0.3     # measured-CP blend of the ZMP (0 = open loop)
    //     zmp_hold_time: 0.03       # [s] the ZMP is held for the last part of a phase
    // (the foot keys of the section, e.g. max_step_length, hip_width, are FootPlanner's)
    // A key that is missing keeps its default from the header. A missing file / section or an
    // invalid value throws std::runtime_error (a constructor cannot return an error code); the
    // caller, e.g. WBCController::on_configure(), catches it and fails cleanly.
    LIPTaskPlanner::LIPTaskPlanner(const std::string &CONFIG_YAML_PATH)
    {
        // load the file; YAML::LoadFile throws YAML::BadFile if it cannot be opened
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(CONFIG_YAML_PATH);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("LIPTaskPlanner: cannot load '" + CONFIG_YAML_PATH + "': " + e.what());
        }

        // all settings are under the "LIPPlanner" key
        const YAML::Node cfg = root["LIPPlanner"];
        if (!cfg || !cfg.IsMap())
        {
            throw std::runtime_error("LIPTaskPlanner: no 'LIPPlanner' section in '" + CONFIG_YAML_PATH + "'");
        }

        // read a value if the key exists, otherwise keep the default; a value that is not a number
        // (e.g. "0.3=") throws with the key name
        const auto read = [&](const char *key, double &value) {
            if (!cfg[key]) return;
            try
            {
                value = cfg[key].as<double>();
            }
            catch (const YAML::Exception &e)
            {
                throw std::runtime_error(std::string("LIPTaskPlanner: '") + key + "' in '" +
                                         CONFIG_YAML_PATH + "' is not a number: " + e.what());
            }
        };

        // the yaml gives the rate, the planner works with the sampling time
        double frequency = 1.0 / this->dt_;
        read("sampling_frequency", frequency);
        read("Tswing", this->T_);
        read("com_height", this->zc_);
        read("cp_feedback_gain", this->cp_feedback_gain_);
        read("zmp_hold_time", this->zmp_hold_time_);

        // check the values before using them (a zero rate or height would divide by zero later)
        if (this->T_ <= 0.0 || this->zc_ <= 0.0 || frequency <= 0.0)
        {
            throw std::runtime_error("LIPTaskPlanner: Tswing, com_height and sampling_frequency must be > 0 "
                                     "(file '" + CONFIG_YAML_PATH + "')");
        }
        if (this->cp_feedback_gain_ < 0.0 || this->cp_feedback_gain_ > 1.0 || this->zmp_hold_time_ < 0.0)
        {
            throw std::runtime_error("LIPTaskPlanner: cp_feedback_gain must be in [0, 1], zmp_hold_time >= 0 "
                                     "(file '" + CONFIG_YAML_PATH + "')");
        }
        this->dt_ = 1.0 / frequency;

        // LIP natural frequency from the CoM height read above (the member initializer used the
        // default zc_)
        this->w_ = std::sqrt(this->g_ / this->zc_);

        // start like WalkPlanner.py: CoM at the origin
        this->reset(Vector2d::Zero());
    }

    // CoM dynamics of the LIP: the CoM is pulled toward the capture point
    Vector2d LIPTaskPlanner::CoM_dynamics(const Vector2d &cp, const Vector2d &com) const
    {
        return this->w_ * (cp - com);
    }

    // CP dynamics: the capture point moves away from the ZMP (unstable part of the LIP)
    Vector2d LIPTaskPlanner::CP_dynamics(const Vector2d &zmp, const Vector2d &cp) const
    {
        return this->w_ * (cp - zmp);
    }

    // integrate the internal CoM one step with the given CP, exactly for a CP constant over dt:
    //   d(com)/dt = w (cp - com)  ->  com(t + dt) = cp + e^{-w dt} (com - cp)
    // (explicit Euler lags behind the exponential and makes the plan drift, see ComputeCP)
    Vector2d LIPTaskPlanner::ComputeCoM(const Vector2d &cp)
    {
        this->com_ = cp + std::exp(-this->w_ * this->dt_) * (this->com_ - cp);
        return this->com_; // returned by value: a copy, no aliasing of the member
    }

    // integrate the internal CP one step with the given ZMP, exactly for a ZMP constant over dt:
    //   d(cp)/dt = w (cp - zmp)  ->  cp(t + dt) = zmp + e^{w dt} (cp - zmp)
    // Explicit Euler grows by (1 + w dt) per step instead of e^{w dt}: over one step (T = 1 s,
    // 500 Hz) 2 % short of e^{wT}, so the CP missed the foothold by ~3 mm per step and the
    // error added up in x.
    Vector2d LIPTaskPlanner::ComputeCP(const Vector2d &zmp)
    {
        this->cp_ = zmp + std::exp(this->w_ * this->dt_) * (this->cp_ - zmp); // analytical solution of the Captured Point dynamics
        return this->cp_;
    }

    // ---------------------------------------------------------------------
    // Walking planner (port of tsid_examples/python/WalkPlanner.py)
    // ---------------------------------------------------------------------

    // restart from the robot's state: CoM at rest (CP = CoM), init phase next
    void LIPTaskPlanner::reset(const Vector2d &com)
    {
        // LIP state at rest
        this->com_ = com;
        this->cp_ = com;
        this->cp_d_ = com;
        this->zmp_d_ = com;

        // walking state: init phase first
        this->done_init_walking_ = false;
        this->phi_com_ = 0.0;
        this->cp_fb_ = com;
        this->t_rem_ = this->T_;
        this->has_meas_ = false;
    }

    void LIPTaskPlanner::setMeasuredState(const Vector2d &com, const Vector2d &cp)
    {
        this->com_meas_ = com;
        this->cp_meas_ = cp;
        this->has_meas_ = true;
    }

    void LIPTaskPlanner::blendTowardMeasured()
    {
        if (!this->has_meas_) return;
        this->com_ += this->cp_feedback_gain_ * (this->com_meas_ - this->com_);
        this->cp_ += this->cp_feedback_gain_ * (this->cp_meas_ - this->cp_);
    }

    Vector2d LIPTaskPlanner::getPredictedEndCP() const
    {
        return this->zmp_d_ + std::exp(this->w_ * this->t_rem_) * (this->cp_fb_ - this->zmp_d_);
    }

    // constant ZMP that brings the CP from cp to cp_d_ in the time t (tsid_examples walk_planner.cpp,
    // Stage 1). CP solution with a constant ZMP p: cp(t) = p + e^{w t} (cp - p), solved for p so
    // that cp(t) = cp_d:  p = (cp_d - b cp) / (1 - b), b = e^{w t}
    Vector2d LIPTaskPlanner::zmpForTargetCP(const Vector2d &cp, double t) const
    {
        const double b = std::exp(this->w_ * t);
        return (this->cp_d_ - b * cp) / (1.0 - b);
    }

    // one planning cycle (WalkPlanner.init_walking / plan_walking, CoM part). The init phase and
    // the steps differ only in cp_target and the ZMP limits (WalkPlanner).
    // Receding horizon (tsid_examples walk_planner.cpp, Stage 1 + 2): every cycle the constant ZMP
    // that brings cp_fb (planned CP blended toward the measured one) to the target in the remaining
    // time of the phase. Undisturbed it is the once-per-phase solution; a CP error (clamped ZMP,
    // a step that ended late, the robot lagging the plan) is spread over the rest of the phase
    void LIPTaskPlanner::planWalking(const Vector2d &cp_target, const Vector2d &zmp_min, const Vector2d &zmp_max)
    {
        // remaining time of the phase from this cycle on, then the phase at the end of the cycle
        const double t_rem = std::max(this->T_ * (1.0 - this->phi_com_), 0.0);
        this->phi_com_ += this->dt_ / this->T_;
        if (this->phi_com_ < 1.0)
        {
            this->planCycle(cp_target, t_rem, zmp_min, zmp_max);
            return;
        }

        // end of the phase: the foot at the target becomes the stance foot, ZMP onto it; the next
        // phase is a step
        this->cp_d_ = cp_target;
        this->zmp_d_ = this->cp_d_;
        this->phi_com_ = 0.0;
        this->done_init_walking_ = true;
        this->ComputeCP(this->zmp_d_);
        this->ComputeCoM(this->cp_);
    }

    void LIPTaskPlanner::planCycle(const Vector2d &cp_target, double t_rem, const Vector2d &zmp_min,
                                   const Vector2d &zmp_max)
    {
        // drive the CP to the target in the remaining time, from the blended CP
        this->t_rem_ = std::max(t_rem, 0.0);
        this->cp_d_ = cp_target;
        this->cp_fb_ = this->has_meas_ ? Vector2d(this->cp_ + this->cp_feedback_gain_ * (this->cp_meas_ - this->cp_))
                                       : this->cp_;

        // inside the limits (e.g. the stance sole): the CP may then miss cp_d a bit. Near the end
        // the ZMP is held (the solution diverges as t_rem -> 0), still kept in the limits of this
        // cycle (they change at a liftoff: double support band -> stance sole)
        if (this->t_rem_ > this->zmp_hold_time_)
        {
            this->zmp_d_ = this->zmpForTargetCP(this->cp_fb_, this->t_rem_);
        }
        this->zmp_d_ = this->zmp_d_.cwiseMax(zmp_min).cwiseMin(zmp_max);

        // integrate the LIP (exact discretization): CP driven by the ZMP, CoM driven by the new CP
        this->ComputeCP(this->zmp_d_);
        this->ComputeCoM(this->cp_);
    }

    void LIPTaskPlanner::setComHeight(double zc)
    {
        if (zc <= 0.0) return;
        this->zc_ = zc;
        this->w_ = std::sqrt(this->g_ / this->zc_);
    }

    // same motion, other frame: every planar point of the state x -> R x + t (the phase is kept)
    void LIPTaskPlanner::transformState(const Eigen::Matrix2d &R, const Vector2d &t)
    {
        this->com_ = R * this->com_ + t;
        this->cp_ = R * this->cp_ + t;
        this->zmp_d_ = R * this->zmp_d_ + t;
        this->cp_d_ = R * this->cp_d_ + t;
        this->cp_fb_ = R * this->cp_fb_ + t;
    }

    // CoM reference at the LIP height
    Vector3d LIPTaskPlanner::getRefCoMPosition() const
    {
        return Vector3d(this->com_.x(), this->com_.y(), this->zc_);
    }

    // CoM velocity from the LIP: d(com)/dt = w (cp - com)
    Vector3d LIPTaskPlanner::getRefCoMVelocity() const
    {
        const Vector2d dcom = this->CoM_dynamics(this->cp_, this->com_);
        return Vector3d(dcom.x(), dcom.y(), 0.0);
    }

    // CoM acceleration from the LIP: d2(com)/dt2 = w (dcp - dcom) = w^2 (com - zmp)
    Vector3d LIPTaskPlanner::getRefCoMAcceleration() const
    {
        const Vector2d ddcom = this->w_ * this->w_ * (this->com_ - this->zmp_d_);
        return Vector3d(ddcom.x(), ddcom.y(), 0.0);
    }
}